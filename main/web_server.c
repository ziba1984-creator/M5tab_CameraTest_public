#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "bsp/m5stack_tab5.h"
#include "app_config.h"
#include "cam_builtin.h"
#include "cam_uvc.h"
#include "display_comp.h"
#include "net_wifi.h"
#include "recorder.h"
#include "web_server.h"

static const char *TAG = "web";

#define SETTINGS_BODY_MAX   512
#define DELETE_BODY_MAX     4096    /* newline-separated Rnnnn.AVI names */
#define TOKEN_LEN           32      /* hex characters of the per-boot request token */
#define PAGE_TOKEN_MARK     "@@TOKEN@@"

static char s_token[TOKEN_LEN + 1];   /* set once by web_server_start() */
#define SEND_CHUNK          (APP_WEB_SEND_KB * 1024)

static httpd_handle_t s_server;

/* One download at a time, served by its own task so the page keeps updating. */
/* Download start/stop protocol. Everything below is changed only under s_dl_mtx
 * (s_dl_abort is also read without it by the send loop):
 *  - rec_get() claims s_dl_busy only while !s_stopping, and resets s_dl_abort and
 *    drains s_dl_done in the same locked section;
 *  - web_server_stop() sets s_stopping first, so no download can start after it
 *    has looked at s_dl_busy;
 *  - whoever releases a claim (download task, or rec_get on a failed start) clears
 *    s_dl_busy and gives s_dl_done in one locked section, so a waiting stop sees
 *    exactly the completion of the download it saw, never a stale one. */
static SemaphoreHandle_t s_dl_mtx;
static bool s_stopping;                 /* web_server_stop() has begun; refuse new downloads */
static bool s_dl_busy;                  /* a download owns (or is about to own) an async request */
static atomic_bool s_dl_abort;
static int s_dl_fd = -1;                /* socket of the running download, -1 until published */
static SemaphoreHandle_t s_dl_done;     /* given when a claim is released */
static char s_dl_name[16];
/* The card reads straight into this buffer by DMA only when it is cache-line
 * aligned; otherwise the driver falls back to one bounce-buffered sector per command. */
static uint8_t *s_dl_buf;

static const char PAGE[] =
"<!doctype html><html lang=ja><meta charset=utf-8>"
"<meta name=viewport content='width=device-width,initial-scale=1'><title>Tab5 Camera</title><style>"
"body{font-family:sans-serif;margin:0 auto;padding:12px 16px;max-width:720px;background:#f2f2f2;color:#222}"
"h1{font-size:20px;margin:4px 0 12px}h2{font-size:16px;margin:0 0 8px}"
"section{background:#fff;border-radius:8px;padding:12px 14px;margin-bottom:12px;box-shadow:0 1px 2px #0002}"
"table{border-collapse:collapse;width:100%}td,th{padding:5px 6px;text-align:left;border-bottom:1px solid #eee;font-size:14px}"
"th{color:#666;font-weight:normal;width:38%}.rec{color:#c00;font-weight:bold}"
"label{display:block;margin:10px 0 3px;font-size:14px}input{width:100%;box-sizing:border-box;padding:7px;font-size:15px}"
"button{margin-top:10px;padding:8px 18px;font-size:15px}small{color:#666}#msg{margin-left:10px}"
"@media(prefers-color-scheme:dark){body{background:#181818;color:#ddd}section{background:#242424}"
"td,th{border-color:#333}th,small{color:#999}a{color:#7ab}}"
"</style><h1>Tab5 Camera</h1>"
"<section><h2>状態</h2><table id=st><tr><td>読み込み中…</td></tr></table></section>"
"<section><h2>録画ファイル</h2><p id=card></p><table id=files></table>"
"<button id=del onclick=del() disabled>選択したファイルを削除</button> <span id=dmsg></span><br>"
"<button onclick=files()>一覧を更新</button><br><small>ダウンロードは1本ずつ、速さは約0.9MB/秒です"
"（5分の録画で約4分）。録画中のファイルはダウンロードできません。</small></section>"
"<section><h2>Wi-Fi 設定</h2><form id=f>"
"<label>Tab5 のアクセスポイントのパスワード（8〜63文字。空欄なら変えない）</label>"
"<input name=ap_pass type=password autocomplete=new-password>"
"<label>接続先のネットワーク名（SSID。空欄にすると接続しない）</label><input name=sta_ssid id=ssid>"
"<label>接続先のパスワード（ネットワーク名が同じなら、空欄で変えない）</label>"
"<input name=sta_pass type=password autocomplete=new-password>"
"<button>保存</button><span id=msg></span></form>"
"<small>保存すると Wi-Fi が一度切れます。つなぎ直してください。</small></section><script>"
"const TK='" PAGE_TOKEN_MARK "';"   /* replaced by the per-boot token when served */
"const $=i=>document.getElementById(i);let P=null,ssidSet=false;"
"const row=(k,v)=>'<tr><th>'+k+'</th><td>'+v+'</td></tr>';"
"const esc=s=>String(s).replace(/[&<>\"']/g,c=>'&#'+c.charCodeAt(0)+';');"
"const mm=s=>Math.floor(s/60)+':'+String(s%60).padStart(2,'0');"
"async function status(){try{const s=await(await fetch('/api/status')).json();const p=P;"
"const dt=p?(s.up-p.up)/1000:0,fps=(a,b)=>dt>0?((a-b)/dt).toFixed(1)+' fps':'…';let r='';"
"r+=row('内蔵カメラ',fps(s.cam,p&&p.cam));"
"r+=row('UVC カメラ',s.uvc.on?s.uvc.w+'x'+s.uvc.h+'、'+fps(s.uvc.dec,p&&p.uvc.dec):'なし');"
"r+=row('画面表示',fps(s.disp,p&&p.disp));"
"const R=s.rec;r+=row('録画',R.state=='rec'?'<span class=rec>● 録画中</span> '+R.file+' '+mm(R.sec)+"
"'（コマ落ち '+R.drop+'、音声欠け '+R.adrop+'）':{idle:'停止中',nocard:'SD カードがありません',error:'書き込みエラーで停止',full:'ファイル番号が上限（R9999）です'}[R.state]);"
"const W=s.wifi;r+=row('アクセスポイント',esc(W.ap_ssid)+'（'+W.ap_ip+'、接続 '+W.ap_clients+' 台）');"
"r+=row('接続先',!W.sta?'設定なし':esc(W.sta_ssid)+(W.sta_conn?'：'+W.sta_ip+'（'+W.rssi+' dBm、ch'+W.ch+'）':'：接続を試みています'));"
"r+=row('C6 ファーム',W.c6||'-');"
"r+=row('空きメモリ','内部 '+s.heap.int+' KB、PSRAM '+s.heap.psram+' KB');"
"r+=row('稼働時間',Math.floor(s.up/60000)+' 分');"
"$('st').innerHTML=r;if(!ssidSet){$('ssid').value=W.sta_ssid;ssidSet=true}P=s}catch(e){}}"
"async function files(){const t=$('files');t.innerHTML='<tr><td>読み込み中…</td></tr>';"
"try{const d=await(await fetch('/api/files')).json();"
"if(!d.card){$('card').textContent='SD カードがありません';t.innerHTML='';return}"
"$('card').textContent='空き '+(d.free/1073741824).toFixed(1)+' GB／'+(d.total/1073741824).toFixed(1)+' GB';"
"d.files.sort((a,b)=>b.n.localeCompare(a.n));"
"t.innerHTML=d.files.length?'<tr><th style=width:2em><input type=checkbox id=all onchange=selAll(this.checked)></th>"
"<th>すべて選択</th><th></th></tr>'+d.files.map(f=>{const a=f.n==d.active;"
"return '<tr><td><input type=checkbox class=sel value='+f.n+' data-s='+f.s+(a?' disabled':'')+' onchange=selUpd()></td><td>'+"
"(a?f.n+' <span class=rec>録画中</span>':'<a href=\"/rec/'+f.n+'\" download>'+f.n+'</a>')+"
"'</td><td>'+(f.s/1048576).toFixed(1)+' MB</td></tr>'}).join('')"
":'<tr><td>録画ファイルはありません</td></tr>';selUpd()}catch(e){t.innerHTML='<tr><td>読み込めませんでした</td></tr>'}}"
"const sels=()=>[...document.querySelectorAll('.sel:checked')];"
"function selAll(v){document.querySelectorAll('.sel:not(:disabled)').forEach(c=>c.checked=v);selUpd()}"
"function selUpd(){const n=sels().length;$('del').disabled=!n;$('del').textContent=n?'選択した '+n+' 件を削除':'選択したファイルを削除'}"
"async function del(){const s=sels();if(!s.length)return;"
"const mb=s.reduce((a,c)=>a+ +c.dataset.s,0)/1048576;"
"if(!confirm(s.length+' 件（合計 '+mb.toFixed(1)+' MB）を SD カードから削除します。\\n元に戻せません。よろしいですか？'))return;"
"$('dmsg').textContent='削除中…';"
"try{const r=await fetch('/api/delete',{method:'POST',headers:{'Content-Type':'text/plain','X-Tab5-Token':TK},body:s.map(c=>c.value).join('\\n')});"
"if(!r.ok){$('dmsg').textContent='削除できません：'+await r.text();return}"
"const d=await r.json();const W={rec:'録画中',dl:'ダウンロード中',gone:'見つからない'};"
"$('dmsg').textContent=d.deleted+' 件を削除しました（'+(d.freed/1048576).toFixed(1)+' MB）'+"
"(d.skipped.length?'。削除しなかったもの：'+d.skipped.map(k=>k.n+'（'+W[k.why]+'）').join('、'):'')}"
"catch(e){$('dmsg').textContent='送信できませんでした'}files()}"
"$('f').onsubmit=async e=>{e.preventDefault();$('msg').textContent='保存中…';"
"try{const r=await fetch('/api/settings',{method:'POST',headers:{'X-Tab5-Token':TK},body:new URLSearchParams(new FormData($('f')))});"
"$('msg').textContent=r.ok?'保存しました。Wi-Fi を再起動します。':'保存できません：'+await r.text()}"
"catch(e){$('msg').textContent='送信できませんでした'}};"
"status();files();setInterval(status,2000);"
"</script>";

/* ---- helpers ------------------------------------------------------------------- */

/* Copy s into dst as JSON string content (without the quotes). */
static void json_escape(char *dst, size_t size, const char *s)
{
    size_t n = 0;
    for (; *s && n + 7 < size; s++) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') {
            dst[n++] = '\\';
            dst[n++] = (char)c;
        } else if (c < 0x20) {
            n += snprintf(dst + n, size - n, "\\u%04x", c);
        } else {
            dst[n++] = (char)c;
        }
    }
    dst[n] = '\0';
}

/* In-place decoding of an application/x-www-form-urlencoded value. */
static void url_decode(char *s)
{
    char *out = s;
    for (; *s; s++) {
        if (*s == '+') {
            *out++ = ' ';
        } else if (*s == '%' && s[1] && s[2]) {
            char hex[3] = { s[1], s[2], 0 };
            *out++ = (char)strtol(hex, NULL, 16);
            s += 2;
        } else {
            *out++ = *s;
        }
    }
    *out = '\0';
}

static bool valid_rec_name(const char *name)
{
    return recorder_parse_name(name, NULL);
}

/* Name of the file being written, or "" when not recording. */
static void active_file(char *dst, size_t size)
{
    recorder_stats_t rec;
    recorder_get_stats(&rec);
    if (rec.state == RECORDER_RECORDING) {
        snprintf(dst, size, "R%04" PRIu32 ".AVI", rec.file_no);
    } else {
        dst[0] = '\0';
    }
}

static esp_err_t send_text(httpd_req_t *req, const char *status, const char *text)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, text);
}

/* ---- handlers ------------------------------------------------------------------ */

/* The page carries the per-boot token in place of PAGE_TOKEN_MARK. */
static esp_err_t page_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const char *mark = strstr(PAGE, PAGE_TOKEN_MARK);
    if (!mark) {
        return httpd_resp_send_500(req);
    }
    esp_err_t err = httpd_resp_send_chunk(req, PAGE, mark - PAGE);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, s_token, HTTPD_RESP_USE_STRLEN);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, mark + strlen(PAGE_TOKEN_MARK), HTTPD_RESP_USE_STRLEN);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, NULL, 0);
    return err;
}

/* Changing requests (delete, settings) must come from this device's own page:
 * the per-boot token travels in a custom header, which a page of another origin
 * can only send after a CORS preflight that this server never answers; and a
 * browser's Origin, when present, must be this host. The confirmation dialog
 * stays a convenience, not the protection. */
static bool request_allowed(httpd_req_t *req)
{
    char got[sizeof(s_token)] = "";
    if (httpd_req_get_hdr_value_str(req, "X-Tab5-Token", got, sizeof(got)) != ESP_OK ||
        strlen(got) != TOKEN_LEN) {
        return false;
    }
    unsigned diff = 0;
    for (int i = 0; i < TOKEN_LEN; i++) {
        diff |= (unsigned)(got[i] ^ s_token[i]);
    }
    if (diff) {
        return false;
    }
    char origin[96], host[80];
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) == ESP_OK) {
        if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK ||
            strncmp(origin, "http://", 7) != 0 || strcmp(origin + 7, host) != 0) {
            return false;
        }
    }
    return true;
}

static esp_err_t status_get(httpd_req_t *req)
{
    frame_mbox_stats_t cam, uvc;
    frame_mbox_get_stats(cam_builtin_mbox(), &cam);
    frame_mbox_get_stats(cam_uvc_mbox(), &uvc);
    display_comp_stats_t disp;
    display_comp_get_stats(&disp);
    cam_uvc_stats_t rx;
    cam_uvc_get_stats(&rx);
    recorder_stats_t rec;
    recorder_get_stats(&rec);
    net_status_t net;
    net_wifi_get_status(&net);

    static const char *const rec_state[] = { "idle", "rec", "nocard", "error", "full" };
    char ssid[3 * 33 + 8];
    json_escape(ssid, sizeof(ssid), net.sta_ssid);
    char c6[24];
    json_escape(c6, sizeof(c6), net.c6_version);
    esp_ip4_addr_t ap_ip = { .addr = net.ap_ip }, sta_ip = { .addr = net.sta_ip };

    char json[1024];
    int n = snprintf(json, sizeof(json),
        "{\"up\":%" PRId64 ",\"cam\":%" PRIu32 ",\"disp\":%" PRIu32 ","
        "\"uvc\":{\"on\":%s,\"w\":%u,\"h\":%u,\"dec\":%" PRIu32 "},"
        "\"rec\":{\"state\":\"%s\",\"file\":\"R%04" PRIu32 ".AVI\",\"sec\":%" PRIu32 ",\"frames\":%" PRIu32
        ",\"drop\":%" PRIu32 ",\"adrop\":%" PRIu32 ",\"bytes\":%" PRIu64 "},"
        "\"wifi\":{\"ap_ssid\":\"%s\",\"ap_ip\":\"" IPSTR "\",\"ap_clients\":%u,\"sta\":%s,\"sta_ssid\":\"%s\","
        "\"sta_conn\":%s,\"sta_ip\":\"" IPSTR "\",\"rssi\":%d,\"ch\":%u,\"c6\":\"%s\"},"
        "\"heap\":{\"int\":%u,\"psram\":%u}}",
        esp_timer_get_time() / 1000, cam.published, disp.flips,
        rx.streaming ? "true" : "false", rx.w, rx.h, uvc.published,
        rec_state[rec.state], rec.file_no, rec.seconds, rec.frames, rec.frames_dropped, rec.audio_dropped, rec.bytes,
        APP_WIFI_AP_SSID, IP2STR(&ap_ip), net.ap_clients, net.sta_configured ? "true" : "false", ssid,
        net.sta_connected ? "true" : "false", IP2STR(&sta_ip), net.sta_rssi, net.channel, c6,
        (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
        (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, n < (int)sizeof(json) ? n : (int)sizeof(json) - 1);
}

static esp_err_t files_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (!recorder_card_acquire(pdMS_TO_TICKS(2000))) {
        return httpd_resp_sendstr(req, "{\"card\":false}");
    }
    uint64_t total = 0, free_bytes = 0;
    esp_vfs_fat_info(BSP_SD_MOUNT_POINT, &total, &free_bytes);
    char active[16];
    active_file(active, sizeof(active));
    char buf[160];
    snprintf(buf, sizeof(buf), "{\"card\":true,\"total\":%" PRIu64 ",\"free\":%" PRIu64 ",\"active\":\"%s\",\"files\":[",
             total, free_bytes, active);
    esp_err_t err = httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    DIR *dir = opendir(APP_REC_DIR);
    bool first = true;
    if (dir) {
        struct dirent *e;
        while (err == ESP_OK && (e = readdir(dir)) != NULL) {
            if (!valid_rec_name(e->d_name)) {
                continue;
            }
            char path[48];
            struct stat st;
            snprintf(path, sizeof(path), APP_REC_DIR "/%.9s", e->d_name);
            if (stat(path, &st) != 0) {
                continue;
            }
            snprintf(buf, sizeof(buf), "%s{\"n\":\"%.9s\",\"s\":%ld}", first ? "" : ",", e->d_name, (long)st.st_size);
            first = false;
            err = httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
        }
        closedir(dir);
    }
    recorder_card_release();
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, "]}", 2);
    }
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, NULL, 0);
    }
    return err;
}

static bool send_all(httpd_req_t *req, const void *data, size_t len)
{
    const char *p = data;
    int timeouts = 0;
    while (len > 0) {
        if (atomic_load(&s_dl_abort)) {
            return false;
        }
        int n = httpd_send(req, p, len);
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 3) {
            continue;
        }
        if (n <= 0) {
            return false;
        }
        timeouts = 0;
        p += n;
        len -= n;
    }
    return true;
}

/* Ends a claim taken by rec_get(): after this, web_server_stop() may free the server. */
static void release_download(void)
{
    xSemaphoreTake(s_dl_mtx, portMAX_DELAY);
    s_dl_fd = -1;
    s_dl_busy = false;
    xSemaphoreGive(s_dl_done);
    xSemaphoreGive(s_dl_mtx);
}

static void download_task(void *arg)
{
    httpd_req_t *req = arg;
    char path[48];
    snprintf(path, sizeof(path), APP_REC_DIR "/%.9s", s_dl_name);
    bool ok = false;
    size_t sent = 0;
    int64_t t0 = esp_timer_get_time();

    if (!recorder_card_acquire(pdMS_TO_TICKS(2000))) {
        send_text(req, "503 Service Unavailable", "SD カードがありません");
        goto done;
    }
    /* read(), not stdio: an unbuffered fread() ran at 75 KB/s against 14.7 MB/s
     * for read() on the same card (2026-11-07). */
    int fd = open(path, O_RDONLY);
    struct stat st;
    if (fd < 0 || fstat(fd, &st) != 0) {
        if (fd >= 0) close(fd);
        recorder_card_release();
        send_text(req, "404 Not Found", "ファイルがありません");
        goto done;
    }
    /* The card lock is held only around each card access, not for the whole
     * transfer, so the file list and deleting keep working during a download.
     * delete_post() refuses the file being downloaded (s_dl_name). */
    recorder_card_release();
    int one = 1;
    xSemaphoreTake(s_dl_mtx, portMAX_DELAY);
    s_dl_fd = httpd_req_to_sockfd(req);
    if (atomic_load(&s_dl_abort)) {     /* stop arrived before the socket was published */
        shutdown(s_dl_fd, SHUT_RDWR);
    }
    xSemaphoreGive(s_dl_mtx);
    setsockopt(httpd_req_to_sockfd(req), IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    char hdr[256];
    int len = snprintf(hdr, sizeof(hdr),
                       "HTTP/1.1 200 OK\r\nContent-Type: video/x-msvideo\r\nContent-Length: %ld\r\n"
                       "Content-Disposition: attachment; filename=\"%s\"\r\nCache-Control: no-store\r\n\r\n",
                       (long)st.st_size, s_dl_name);
    ok = send_all(req, hdr, len);
    while (ok && sent < (size_t)st.st_size) {
        if (!recorder_card_acquire(pdMS_TO_TICKS(10000))) {
            ok = false;     /* card removed (unmounted) or held too long */
            break;
        }
        int n = read(fd, s_dl_buf, SEND_CHUNK);
        recorder_card_release();
        if (n <= 0) {
            ok = false;
            break;
        }
        ok = send_all(req, s_dl_buf, n);
        sent += ok ? n : 0;
    }
    /* Wait as long as it takes: a busy card (a long delete, say) must not leak the
     * descriptor (max_files is 4). With portMAX_DELAY, false means unmounted only,
     * and then the descriptor went away with the card. */
    if (recorder_card_acquire(portMAX_DELAY)) {
        close(fd);
        recorder_card_release();
    }
    int64_t us = esp_timer_get_time() - t0;
    if (ok) {
        ESP_LOGI(TAG, "download %s: %u KB in %" PRId64 " ms = %.2f Mbit/s", s_dl_name, (unsigned)(sent / 1024),
                 us / 1000, us > 0 ? sent * 8.0 / us : 0);
    } else {
        ESP_LOGW(TAG, "download %s stopped after %u KB", s_dl_name, (unsigned)(sent / 1024));
        /* The response is incomplete: close the connection so the browser notices. */
        httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
    }
done:
    httpd_req_async_handler_complete(req);  /* last use of req and its server */
    release_download();
    vTaskDelete(NULL);
}

static esp_err_t rec_get(httpd_req_t *req)
{
    const char *name = req->uri + strlen("/rec/");
    char clean[16];
    strlcpy(clean, name, sizeof(clean));
    char *q = strchr(clean, '?');
    if (q) *q = '\0';
    if (!valid_rec_name(clean)) {
        return send_text(req, "404 Not Found", "ファイルがありません");
    }
    char active[16];
    active_file(active, sizeof(active));
    if (strcmp(clean, active) == 0) {
        return send_text(req, "409 Conflict", "録画中のファイルです。録画を止めてからダウンロードしてください");
    }
    xSemaphoreTake(s_dl_mtx, portMAX_DELAY);
    bool stopping = s_stopping, busy = s_dl_busy;
    if (!stopping && !busy) {
        s_dl_busy = true;
        atomic_store(&s_dl_abort, false);   /* safe: no stop has begun (s_stopping is false) */
        s_dl_fd = -1;
        xSemaphoreTake(s_dl_done, 0);       /* no claim is outstanding, so any token is stale */
        strlcpy(s_dl_name, clean, sizeof(s_dl_name));
    }
    xSemaphoreGive(s_dl_mtx);
    if (stopping) {
        return send_text(req, "503 Service Unavailable", "Web ページを停止中です");
    }
    if (busy) {
        return send_text(req, "503 Service Unavailable", "別のファイルをダウンロード中です");
    }
    /* From here this request holds the claim: every path below must release it once. */
    httpd_req_t *copy = NULL;
    if (httpd_req_async_handler_begin(req, &copy) != ESP_OK) {
        release_download();
        return send_text(req, "500 Internal Server Error", "開始できませんでした");
    }
    if (xTaskCreate(download_task, "web_dl", 4096, copy, APP_WEB_PRIO, NULL) != pdPASS) {
        send_text(copy, "500 Internal Server Error", "開始できませんでした");
        httpd_req_async_handler_complete(copy);
        release_download();
    }
    return ESP_OK;
}

/* POST /api/delete, body: Rnnnn.AVI names separated by newlines. Refuses the file
 * being recorded (and anything newer while recording) and the file being
 * downloaded; reports each refusal. */
static esp_err_t delete_post(httpd_req_t *req)
{
    if (!request_allowed(req)) {
        ESP_LOGW(TAG, "delete refused: missing or wrong token, or foreign origin");
        return send_text(req, "403 Forbidden", "このページから操作してください（ページを再読み込みしてください）");
    }
    if (req->content_len == 0 || req->content_len > DELETE_BODY_MAX) {
        return send_text(req, "400 Bad Request", "削除するファイルの指定が不正です");
    }
    char *body = malloc(req->content_len + 1);
    if (!body) {
        return send_text(req, "500 Internal Server Error", "メモリが足りません");
    }
    size_t got = 0;
    while (got < req->content_len) {
        int n = httpd_req_recv(req, body + got, req->content_len - got);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (n <= 0) {
            free(body);
            return ESP_FAIL;
        }
        got += n;
    }
    body[got] = '\0';

    if (!recorder_card_acquire(pdMS_TO_TICKS(5000))) {
        free(body);
        return send_text(req, "503 Service Unavailable", "SD カードがありません");
    }
    unsigned deleted = 0;
    uint64_t freed = 0;
    char out[1024];
    int o = snprintf(out, sizeof(out), "{\"skipped\":[");
    bool first = true;
    char *save = NULL;
    for (char *name = strtok_r(body, "\r\n", &save); name; name = strtok_r(NULL, "\r\n", &save)) {
        uint32_t no;
        const char *why = NULL;
        if (!recorder_parse_name(name, &no)) {
            continue;   /* not a recording name: never touched, not reported */
        }
        /* While recording, the current file and any newer number are off limits
         * (the writer opens the next number by itself). */
        recorder_stats_t rec;
        recorder_get_stats(&rec);
        if (rec.state == RECORDER_RECORDING && no >= rec.file_no) {
            why = "rec";
        }
        if (!why) {
            xSemaphoreTake(s_dl_mtx, portMAX_DELAY);
            bool downloading = s_dl_busy && strcmp(s_dl_name, name) == 0;
            xSemaphoreGive(s_dl_mtx);
            if (downloading) {
                why = "dl";
            }
        }
        char path[48];
        snprintf(path, sizeof(path), APP_REC_DIR "/%.9s", name);
        if (!why) {
            struct stat st;
            long size = stat(path, &st) == 0 ? (long)st.st_size : 0;
            if (unlink(path) == 0) {
                deleted++;
                freed += size;
                ESP_LOGI(TAG, "deleted %s (%ld bytes)", name, size);
                continue;
            }
            why = "gone";
        }
        if (o < (int)sizeof(out) - 40) {
            o += snprintf(out + o, sizeof(out) - o, "%s{\"n\":\"%.9s\",\"why\":\"%s\"}", first ? "" : ",", name, why);
            first = false;
        }
    }
    recorder_card_release();
    free(body);
    snprintf(out + o, sizeof(out) - o, "],\"deleted\":%u,\"freed\":%" PRIu64 "}", deleted, freed);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, out);
}

static esp_err_t settings_post(httpd_req_t *req)
{
    if (!request_allowed(req)) {
        ESP_LOGW(TAG, "settings refused: missing or wrong token, or foreign origin");
        return send_text(req, "403 Forbidden", "このページから操作してください（ページを再読み込みしてください）");
    }
    if (req->content_len == 0 || req->content_len > SETTINGS_BODY_MAX) {
        return send_text(req, "400 Bad Request", "入力が長すぎます");
    }
    char body[SETTINGS_BODY_MAX + 1];
    size_t got = 0;
    while (got < req->content_len) {
        int n = httpd_req_recv(req, body + got, req->content_len - got);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (n <= 0) {
            return ESP_FAIL;
        }
        got += n;
    }
    body[got] = '\0';

    char ap_pass[3 * 64 + 1] = "", sta_ssid[3 * 33 + 1] = "", sta_pass[3 * 64 + 1] = "";
    bool has_ssid = httpd_query_key_value(body, "sta_ssid", sta_ssid, sizeof(sta_ssid)) == ESP_OK;
    httpd_query_key_value(body, "ap_pass", ap_pass, sizeof(ap_pass));
    httpd_query_key_value(body, "sta_pass", sta_pass, sizeof(sta_pass));
    memset(body, 0, sizeof(body));
    url_decode(ap_pass);
    url_decode(sta_ssid);
    url_decode(sta_pass);

    net_status_t net;
    net_wifi_get_status(&net);
    bool same_ssid = has_ssid && strcmp(sta_ssid, net.sta_ssid) == 0;
    /* Empty password: keep it for the same network, an open network for a new one. */
    const char *pass_arg = sta_pass[0] ? sta_pass : same_ssid ? NULL : "";
    esp_err_t err = net_wifi_save_settings(ap_pass[0] ? ap_pass : NULL, has_ssid ? sta_ssid : NULL,
                                           has_ssid ? pass_arg : NULL);
    memset(ap_pass, 0, sizeof(ap_pass));
    memset(sta_pass, 0, sizeof(sta_pass));
    if (err == ESP_ERR_INVALID_ARG) {
        return send_text(req, "400 Bad Request",
                         "パスワードは8〜63文字の半角英数字・記号、ネットワーク名は32バイト以内にしてください");
    }
    if (err != ESP_OK) {
        return send_text(req, "500 Internal Server Error", "保存に失敗しました");
    }
    return send_text(req, "200 OK", "OK");
}

/* ---- public -------------------------------------------------------------------- */

esp_err_t web_server_start(void)
{
    if (s_server) {
        /* Left running by a stop that timed out: in use again, accept downloads. */
        xSemaphoreTake(s_dl_mtx, portMAX_DELAY);
        s_stopping = false;
        xSemaphoreGive(s_dl_mtx);
        return ESP_OK;
    }
    if (!s_dl_buf) {
        s_dl_buf = heap_caps_aligned_alloc(64, SEND_CHUNK, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        if (!s_dl_buf) {
            return ESP_ERR_NO_MEM;
        }
    }
    /* Each created on its own when missing, so a start that failed half way is
     * retried safely (no NULL mutex in use, nothing created twice). */
    if (!s_dl_done) {
        s_dl_done = xSemaphoreCreateBinary();
    }
    if (!s_dl_mtx) {
        s_dl_mtx = xSemaphoreCreateMutex();
    }
    if (!s_dl_done || !s_dl_mtx) {
        return ESP_ERR_NO_MEM;
    }
    if (!s_token[0]) {      /* once per boot: pages loaded before a reboot stop working */
        uint8_t rnd[TOKEN_LEN / 2];
        esp_fill_random(rnd, sizeof(rnd));
        for (int i = 0; i < (int)sizeof(rnd); i++) {
            snprintf(s_token + 2 * i, 3, "%02x", rnd[i]);
        }
    }
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = APP_WEB_STACK;
    cfg.task_priority = APP_WEB_PRIO;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;
    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) {
        s_server = NULL;
        return err;
    }
    xSemaphoreTake(s_dl_mtx, portMAX_DELAY);
    s_stopping = false;
    xSemaphoreGive(s_dl_mtx);
    const httpd_uri_t uris[] = {
        { .uri = "/",             .method = HTTP_GET,  .handler = page_get },
        { .uri = "/api/status",   .method = HTTP_GET,  .handler = status_get },
        { .uri = "/api/files",    .method = HTTP_GET,  .handler = files_get },
        { .uri = "/rec/*",        .method = HTTP_GET,  .handler = rec_get },
        { .uri = "/api/settings", .method = HTTP_POST, .handler = settings_post },
        { .uri = "/api/delete",   .method = HTTP_POST, .handler = delete_post },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_server, &uris[i]);
    }
    ESP_LOGI(TAG, "web page on port %d", cfg.server_port);
    return ESP_OK;
}

esp_err_t web_server_stop(void)
{
    if (!s_server) {
        return ESP_OK;
    }
    /* From here on rec_get() refuses new downloads, so s_dl_busy can only go
     * from true to false. A running download owns an async request of this
     * server: free the server only after that request has been completed.
     * Shutting the socket down ends a send blocked on a stalled client. */
    xSemaphoreTake(s_dl_mtx, portMAX_DELAY);
    s_stopping = true;
    bool busy = s_dl_busy;
    if (busy) {
        atomic_store(&s_dl_abort, true);
        if (s_dl_fd >= 0) {
            shutdown(s_dl_fd, SHUT_RDWR);
        }
    }
    xSemaphoreGive(s_dl_mtx);
    if (busy && xSemaphoreTake(s_dl_done, pdMS_TO_TICKS(15000)) != pdTRUE) {
        /* Never free a server a task may still use. It stays up (and keeps
         * refusing downloads) until a later stop succeeds. */
        ESP_LOGE(TAG, "download did not finish; web server left running");
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = httpd_stop(s_server);
    if (err != ESP_OK) {
        /* The server may still be running: keep its handle (and the stopping
         * state) so the caller can report it and a later stop can retry. */
        ESP_LOGE(TAG, "httpd_stop failed (%s); web server left running", esp_err_to_name(err));
        return err;
    }
    s_server = NULL;
    return ESP_OK;
}

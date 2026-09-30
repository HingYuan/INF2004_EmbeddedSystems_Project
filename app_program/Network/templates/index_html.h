#ifndef WEB_INDEX_HTML_H
#define WEB_INDEX_HTML_H

static const char web_index_html[] =
"<!doctype html>"
"<html lang=\"en\"><head>"
"<meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
"<title>Pico W</title>"
"<style>body{margin:2rem;font:1rem system-ui,sans-serif;color:#17324d;background:#eef5f7}"
"main{max-width:38rem;padding:2rem;background:white;border:1px solid #c7d9df;border-radius:8px}"
"h1{margin-top:0}code{color:#087f8c}</style></head>"
"<body><main><h1>Pico W web server</h1>"
"<p>The &micro;T-Kernel network service is running.</p>"
"<p>Listening on <code>port 8080</code>.</p>"
"</main></body></html>";

#define WEB_INDEX_HTML_LEN (sizeof(web_index_html) - 1U)

#endif

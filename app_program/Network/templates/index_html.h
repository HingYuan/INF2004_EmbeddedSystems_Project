#ifndef WEB_INDEX_HTML_H
#define WEB_INDEX_HTML_H

static const char web_index_html[] =
"<!doctype html>"
"<html lang=\"en\"><head>"
"<meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
"<title>Pico W</title>"
"<style>body{margin:2rem;font:1rem system-ui,sans-serif;color:#17324d;background:#eef5f7}"
"main{max-width:46rem;padding:2rem;background:white;border:1px solid #c7d9df;border-radius:8px}"
"h1{margin-top:0}section{margin-top:1.5rem;padding-top:1rem;border-top:1px solid #d8e5e8}"
"dt{font-weight:700;margin-top:.7rem}dd{margin:.2rem 0;word-break:break-word}"
"pre{white-space:pre-wrap;background:#f2f7f8;padding:1rem;border-radius:4px;min-height:3rem}"
".ok{color:#087f4b}.bad{color:#a52a2a}</style></head>"
"<body><main><h1>Pico W MQTT dashboard</h1>"
"<p>The &micro;T-Kernel network service is running on port <code>8080</code>.</p>"
"<section><dl><dt>Broker</dt><dd id=state>Connecting...</dd>"
"<dt>Messages received</dt><dd id=count>0</dd>"
"<dt>Latest topic</dt><dd id=topic>-</dd></dl>"
"<dt>Latest payload</dt><dd><pre id=payload>-</pre></dd></section></main>"
"<script>async function refresh(){try{const r=await fetch('/api/mqtt',{cache:'no-store'});"
"const d=await r.json();const s=document.getElementById('state');"
"s.textContent=d.enabled?(d.connected?'Connected':'Waiting for broker'):'MQTT disabled';"
"s.className=d.connected?'ok':'bad';document.getElementById('count').textContent=d.messages;"
"document.getElementById('topic').textContent=d.topic||'-';"
"document.getElementById('payload').textContent=d.payload||'-';}catch(e){"
"document.getElementById('state').textContent='Dashboard unavailable';}}"
"refresh();setInterval(refresh,1000);</script></body></html>";

#define WEB_INDEX_HTML_LEN (sizeof(web_index_html) - 1U)

#endif

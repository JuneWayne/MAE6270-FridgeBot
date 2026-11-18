pdfjsLib.GlobalWorkerOptions.workerSrc = 'https://cdnjs.cloudflare.com/ajax/libs/pdf.js/3.11.174/pdf.worker.min.js';
const stage = document.getElementById('stage');
const views = {};   // deck id -> {deck, el, slides, doc}
let current = null;

// a slide fits the visible area: full width, and never taller than the screen
function slideSize(aspect) {
  const fs = document.fullscreenElement === stage;
  const W = stage.clientWidth - (fs ? 0 : 32), H = stage.clientHeight - (fs ? 0 : 32);
  const w = Math.max(200, Math.min(W, H * aspect));
  return [w, w / aspect];
}

function layout(v) {
  const [w, h] = slideSize(v.deck.aspect);
  stage.style.setProperty('--h', h + 'px');
  for (const s of v.slides) {
    s.el.style.width = w + 'px'; s.el.style.height = h + 'px';
    if (s.drawnAt && Math.abs(s.drawnAt - w) > 40) { s.drawnAt = 0; if (s.visible) draw(v, s); }
  }
}

async function draw(v, s) {
  if (s.busy || s.drawnAt || !v.doc) return;
  s.busy = true;
  try {
    const page = await v.doc.getPage(s.n);
    const w = s.el.clientWidth, base = page.getViewport({ scale: 1 });
    const vp = page.getViewport({ scale: Math.min(4, w * (window.devicePixelRatio || 1) / base.width) });
    const c = document.createElement('canvas');
    c.width = Math.round(vp.width); c.height = Math.round(vp.height);
    await page.render({ canvasContext: c.getContext('2d'), viewport: vp }).promise;
    if (s.canvas) s.canvas.replaceWith(c); else s.el.prepend(c);
    s.canvas = c; s.drawnAt = w;
  } finally { s.busy = false; }
}

function setVideos(s, on) {
  for (const vid of s.videos) {
    if (on) { if (!vid.src) vid.src = vid.dataset.src; vid.play().catch(() => {}); }
    else vid.pause();
  }
}

function build(deck) {
  const el = document.createElement('section');
  el.className = 'deck'; el.hidden = true;
  const v = { deck, el, slides: [], doc: null };
  for (let n = 1; n <= deck.pages; n++) {
    const s = { n, el: document.createElement('div'), videos: [] };
    s.el.className = 'slide'; s.el.id = deck.id + '-' + n; s.el.setAttribute('aria-label', 'Slide ' + n);
    for (const m of deck.media.filter(m => m.page === n)) {
      const vid = document.createElement('video');
      Object.assign(vid, { muted: true, loop: true, playsInline: true, preload: 'none' });
      vid.setAttribute('muted', ''); vid.dataset.src = ROOT + m.src;
      Object.assign(vid.style, { left: m.x * 100 + '%', top: m.y * 100 + '%', width: m.w * 100 + '%', height: m.h * 100 + '%' });
      // text drawn on top of the gif in the slide: leave that strip to the page below
      if (m.clip) vid.style.clipPath = `inset(${m.clip[0] * 100}% 0 ${m.clip[1] * 100}% 0)`;
      s.el.append(vid); s.videos.push(vid);
    }
    el.append(s.el); v.slides.push(s);
  }
  stage.append(el);
  const io = new IntersectionObserver(es => {
    for (const e of es) {
      const s = v.slides.find(s => s.el === e.target);
      s.visible = e.isIntersecting;
      if (s.visible) draw(v, s);
      setVideos(s, s.visible && current === v);
    }
  }, { root: stage, rootMargin: '100% 0px' });
  v.io = io;
  pdfjsLib.getDocument(ROOT + deck.pdf).promise.then(doc => {
    v.doc = doc;
    for (const s of v.slides) if (s.visible) draw(v, s);
  }).catch(() => { el.innerHTML = '<p class="msg">The slides could not load. Use Download PDF.</p>'; });
  views[deck.id] = v;
  return v;
}

function show(id) {
  const v = views[id] || views[Object.keys(views)[0]];
  if (current === v) return;
  if (current) { current.el.hidden = true; current.slides.forEach(s => setVideos(s, false)); }
  current = v; v.el.hidden = false;
  document.getElementById('title').textContent = v.deck.title;
  const dl = document.getElementById('pdf'); dl.href = ROOT + v.deck.pdf; dl.download = v.deck.id + '.pdf';
  layout(v); stage.scrollTop = 0;
  if (!v.observed) { v.slides.forEach(s => v.io.observe(s.el)); v.observed = true; }
  else v.slides.forEach(s => setVideos(s, s.visible));
}

function step(d) {
  if (!current) return;
  const top = stage.scrollTop + stage.clientHeight / 2;
  let i = current.slides.findIndex(s => s.el.offsetTop + s.el.offsetHeight > top);
  i = Math.max(0, Math.min(current.slides.length - 1, (i < 0 ? 0 : i) + d));
  current.slides[i].el.scrollIntoView({ block: 'center' });
}

fetch(ROOT + 'decks.json').then(r => r.json()).then(decks => {
  const d = decks.find(d => d.id === DECK);
  build(d); show(d.id);
});
let t; addEventListener('resize', () => { clearTimeout(t); t = setTimeout(() => current && layout(current), 150); });
document.addEventListener('fullscreenchange', () => current && layout(current));
document.getElementById('fs').onclick = () => (stage.requestFullscreen || stage.webkitRequestFullscreen).call(stage);
addEventListener('keydown', e => {
  if (document.fullscreenElement !== stage) return;
  if (['ArrowRight', 'ArrowDown', 'PageDown', ' '].includes(e.key)) { e.preventDefault(); step(1); }
  if (['ArrowLeft', 'ArrowUp', 'PageUp'].includes(e.key)) { e.preventDefault(); step(-1); }
});

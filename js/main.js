/* ===== Theme ===== */
const root = document.documentElement;

document.getElementById("theme-toggle").addEventListener("click", () => {
  const theme = root.dataset.theme === "dark" ? "light" : "dark";
  root.dataset.theme = theme;
  try {
    localStorage.setItem("theme", theme);
  } catch {}
  document.dispatchEvent(new Event("themechange")); // demos re-read their colours
});

/* ===== Mobile nav ===== */
const nav = document.getElementById("nav");
const navToggle = document.getElementById("nav-toggle");

function setMenu(open) {
  nav.classList.toggle("open", open);
  navToggle.setAttribute("aria-expanded", open);
  navToggle.firstElementChild.className = `bx ${open ? "bx-x" : "bx-menu"}`;
}
navToggle.addEventListener("click", () => setMenu(!nav.classList.contains("open")));
nav.addEventListener("click", (e) => e.target.closest("a") && setMenu(false));

/* ===== Active nav link + reveal on scroll ===== */
const links = [...nav.querySelectorAll("a")];
const spy = new IntersectionObserver(
  (entries) =>
    entries.forEach((e) => {
      if (e.isIntersecting) links.forEach((a) => a.classList.toggle("active", a.hash === `#${e.target.id}`));
    }),
  { rootMargin: "-40% 0px -55% 0px" }
);
document.querySelectorAll("main section[id]").forEach((s) => spy.observe(s));

const reveal = new IntersectionObserver(
  (entries) =>
    entries.forEach((e) => {
      if (!e.isIntersecting) return;
      e.target.classList.add("visible");
      reveal.unobserve(e.target);
    }),
  { rootMargin: "0px 0px -8% 0px" }
);
document.querySelectorAll(".reveal").forEach((el) => reveal.observe(el));

/* ===== Demo tabs (live / video) ===== */
document.querySelectorAll(".demo__tabs").forEach((tabs) => {
  const demo = tabs.closest(".demo");
  tabs.addEventListener("click", (e) => {
    const tab = e.target.closest("[data-tab]");
    if (!tab) return;
    tabs.querySelectorAll("[data-tab]").forEach((t) => t.setAttribute("aria-selected", t === tab));
    demo.querySelectorAll("[data-panel]").forEach((p) => (p.hidden = p.dataset.panel !== tab.dataset.tab));
    demo.classList.toggle("is-video", tab.dataset.tab === "video");
    const video = demo.querySelector("video");
    tab.dataset.tab === "video" ? video.play() : video.pause();
  });
});

/* ===== Live demos ===== */
// LIVE DEMO OFF: the demo scripts are commented out in index.html
// document.querySelectorAll("[data-demo]").forEach((el) => Demos[el.dataset.demo](el));

/* ===== Misc ===== */
const copyBtn = document.getElementById("copy-email");
copyBtn.addEventListener("click", async () => {
  try {
    await navigator.clipboard.writeText(copyBtn.dataset.email);
    copyBtn.querySelector("span").textContent = "Copied";
    setTimeout(() => (copyBtn.querySelector("span").textContent = "Copy"), 1600);
  } catch {
    location.href = `mailto:${copyBtn.dataset.email}`;
  }
});

document.getElementById("year").textContent = new Date().getFullYear();

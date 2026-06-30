import { createHighlighter } from 'shiki';
async function test() {
  const hl = await createHighlighter({ themes: ['dark-plus'], langs: ['cpp'] });
  console.log(hl.codeToHtml('a\n\nb', { lang: 'cpp', theme: 'dark-plus' }));
}
test();

import { createHighlighter } from 'shiki';
import * as cheerio from 'cheerio';
import { globSync } from 'glob';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

async function processHtmlFiles() {
    console.log('Initializing Shiki...');
    const highlighter = await createHighlighter({
        themes: ['dark-plus'],
        langs: ['cpp', 'bash', 'yaml', 'json', 'python', 'go', 'javascript', 'html', 'css', 'shell', 'plaintext', 'text']
    });

    const publicDir = path.resolve(__dirname, '../public');
    const htmlFiles = globSync('**/*.html', { cwd: publicDir, absolute: true });

    console.log(`Found ${htmlFiles.length} HTML files to process.`);

    let processedCount = 0;
    for (const file of htmlFiles) {
        const html = fs.readFileSync(file, 'utf8');
        const $ = cheerio.load(html);
        let modified = false;

        const codeBlocks = $('.highlight code[data-lang]');

        if (codeBlocks.length > 0) {
            codeBlocks.each((_, el) => {
                let lang = $(el).attr('data-lang');
                if (lang === 'sh') lang = 'shell';
                if (!highlighter.getLoadedLanguages().includes(lang)) {
                    lang = 'plaintext';
                }
                
                let codeText = $(el).text();
                // Strip the trailing newline that Hugo adds, to prevent an extra empty line
                if (codeText.endsWith('\n')) {
                    codeText = codeText.slice(0, -1);
                }

                try {
                    const shikiHtml = highlighter.codeToHtml(codeText, {
                        lang: lang,
                        theme: 'dark-plus'
                    });

                    const shikiDoc = cheerio.load(shikiHtml);
                    const innerSpans = shikiDoc('code').html();

                    $(el).html(innerSpans);
                    modified = true;
                } catch (e) {
                    console.warn(`Failed to highlight block in ${file} with lang ${lang}:`, e.message);
                }
            });
        }

        if (modified) {
            fs.writeFileSync(file, $.html(), 'utf8');
            processedCount++;
        }
    }
    console.log(`Shiki post-processing complete! Updated ${processedCount} files.`);
}

processHtmlFiles().catch(console.error);

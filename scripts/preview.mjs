import http from 'node:http';
import { createHash } from 'node:crypto';
import { tmpdir } from 'node:os';
import { existsSync, readdirSync, statSync } from 'node:fs';
import { readFile, stat } from 'node:fs/promises';
import path from 'node:path';
import { build, root } from './build.mjs';

const port = Number(process.env.PORT || 1313);
const host = process.env.HOST || '0.0.0.0';
const baseURL = process.env.BASE_URL || `http://localhost:${port}/`;
// Build locally on Windows: repeated Hugo writes to SMB shares can fail.
// A project-specific folder also keeps different checkouts separate.
const output = process.platform === 'win32'
    ? path.join(tmpdir(), 'pages-preview', createHash('sha256').update(root).digest('hex').slice(0, 16))
    : path.join(root, '.preview');
let revision = 0;
let pending = true;
let building;
let buildError;

async function rebuild() {
    pending = true;
    if (building) return building;
    building = (async () => {
        while (pending) {
            pending = false;
            try {
                await build(['--baseURL', baseURL], { destination: output });
                buildError = undefined;
                revision++;
            } catch (error) {
                buildError = error;
                console.error(error);
            }
        }
    })();
    await building;
    building = undefined;
}

const types = { '.html': 'text/html; charset=utf-8', '.css': 'text/css', '.js': 'text/javascript', '.json': 'application/json', '.svg': 'image/svg+xml', '.png': 'image/png', '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.webp': 'image/webp', '.ico': 'image/x-icon', '.woff2': 'font/woff2', '.xml': 'application/xml', '.txt': 'text/plain' };
const server = http.createServer(async (req, res) => {
    try {
        await building;
        const pathname = decodeURIComponent(new URL(req.url, baseURL).pathname);
        res.setHeader('Cache-Control', 'no-store');
        if (pathname === '/__preview_revision') {
            res.end(String(revision));
            return;
        }
        if (buildError) {
            res.writeHead(503, { 'Content-Type': 'text/plain; charset=utf-8' });
            res.end('Build failed. Check the terminal, fix the source, and save again.');
            return;
        }
        let file = path.resolve(output, '.' + pathname);
        if (!file.startsWith(output + path.sep) && file !== output) {
            res.writeHead(403).end();
            return;
        }
        if ((await stat(file)).isDirectory()) {
            if (!pathname.endsWith('/')) {
                res.writeHead(301, { Location: pathname + '/' }).end();
                return;
            }
            file = path.join(file, 'index.html');
        }
        let content = await readFile(file);
        const ext = path.extname(file);
        if (ext === '.html') {
            content = content.toString().replace('</body>', `<script>setInterval(async()=>{try{const r=await fetch('/__preview_revision');if(r.ok&&(await r.text())!==${JSON.stringify(String(revision))})location.reload()}catch{}},1000)</script></body>`);
        }
        res.setHeader('Content-Type', types[ext] || 'application/octet-stream');
        res.end(content);
    } catch {
        res.writeHead(404).end('Not found');
    }
});

server.on('error', error => { console.error(error); process.exit(1); });
// Windows can notify on reads too. Compare source metadata to avoid build loops.
function fingerprint() {
    const entries = [];
    function visit(target) {
        if (!existsSync(target)) return;
        const info = statSync(target);
        if (info.isDirectory()) {
            for (const name of readdirSync(target).sort()) {
                if (name !== '.git') visit(path.join(target, name));
            }
        } else entries.push(`${target}:${info.size}:${info.mtimeMs}`);
    }
    for (const name of ['hugo.toml', 'content', 'assets', 'layouts', 'static', 'themes', 'archetypes', 'data', 'i18n', 'config', 'scripts']) {
        visit(path.join(root, name));
    }
    return entries.join('\n');
}
let previous = fingerprint();
setInterval(() => {
    try {
        const current = fingerprint();
        if (current !== previous) {
            previous = current;
            void rebuild();
        }
    } catch (error) { console.error('Unable to inspect sources:', error.message); }
}, 1000);

await rebuild();
server.listen(port, host, () => console.log(`Preview ready: ${baseURL} (listening on ${host}:${port}, Hugo + Shiki). Press Ctrl+C to stop.`));

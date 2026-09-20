import { spawn } from 'node:child_process';
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

export const root = fileURLToPath(new URL('../', import.meta.url));

function run(command, args) {
    return new Promise((resolve, reject) => {
        const child = spawn(command, args, { cwd: root, stdio: 'inherit' });
        child.on('error', reject);
        child.on('exit', code => code === 0 ? resolve() : reject(new Error(`${command} exited with ${code}`)));
    });
}

export async function build(args = [], { destination } = {}) {
    // Preserve Hugo's CLI destination option and pass the same directory to Shiki.
    const index = args.findLastIndex(arg => arg === '--destination' || arg === '-d' || arg.startsWith('--destination='));
    const cliDestination = index < 0 ? undefined : (args[index].startsWith('--destination=') ? args[index].slice('--destination='.length) : args[index + 1]);
    const output = path.resolve(root, destination || cliDestination || 'public');
    // WinGet's installation may not yet be on the current terminal's PATH.
    const wingetHugo = path.join(process.env.LOCALAPPDATA || '', 'Microsoft/WinGet/Links/hugo.exe');
    const hugo = process.env.HUGO_BINARY || (process.platform === 'win32' && existsSync(wingetHugo) ? wingetHugo : 'hugo');
    await run(hugo, ['--buildDrafts', '--gc', '--cleanDestinationDir', ...args, '--destination', output]);
    await run(process.execPath, ['scripts/shiki-highlight.mjs', output]);
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
    build(process.argv.slice(2)).catch(error => {
        console.error(error);
        process.exitCode = 1;
    });
}

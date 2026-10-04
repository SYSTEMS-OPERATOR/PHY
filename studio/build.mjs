import { build } from 'esbuild';
import { readFile, writeFile } from 'node:fs/promises';
await build({ entryPoints: ['src/app.js'], bundle: true, minify: true, format: 'iife',
  outfile: 'app.bundle.js', target: ['es2022'], legalComments: 'inline' });
const bundle = await readFile('app.bundle.js', 'utf8');
await writeFile('app.bundle.js', bundle.split('\n').map(line => line.trimEnd()).join('\n'));

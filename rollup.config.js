import typescript from '@rollup/plugin-typescript';
import terser from '@rollup/plugin-terser';
import {string} from 'rollup-plugin-string';

const input = 'src/ammo-es.ts';

const plugins = [
    string({include: '**/*.base64'}),
    typescript({tsconfig: 'tsconfig.json'}),
    terser({
        compress: {
            dead_code: true, // 是否移除死碼
            unused: true, // 是否移除未使用的變數
            drop_console: false // 是否移除 console
        },
        mangle: false
    })
];

export default [
    {
        input,
        plugins,
        output: {
            file: 'dist/ammo-es.js',
            format: 'esm',
            sourcemap: true
        }
    },
    {
        input,
        plugins,
        output: {
            file: 'dist/ammo-es.umd.js',
            format: 'umd',
            name: 'Ammo',
            sourcemap: true
        }
    }
];

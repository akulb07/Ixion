import { build } from "esbuild";
import { fileURLToPath } from "node:url";
import { readFile, mkdir, writeFile } from "node:fs/promises";
import { createRequire } from "node:module";
import path from "node:path";
import { createHash } from "node:crypto";
// Some Windows sandboxes deny Go's ancestor-directory scans. In that environment,
// resolve and read the same project files through Node's filesystem API.
const nodeFiles = {
  name: "node-files",
  setup(builder) {
    builder.onResolve({ filter: /.*/ }, (args) => {
      const from = args.importer || path.join(process.cwd(), "build.mjs");
      const resolver = createRequire(from);
      let target;
      if (args.path.startsWith(".")) {
        const base = path.resolve(path.dirname(from), args.path);
        for (const suffix of ["", ".tsx", ".ts", ".js", ".mjs", ".css"]) {
          try {
            target = resolver.resolve(base + suffix);
            break;
          } catch {}
        }
        if (!target)
          throw new Error(`Cannot resolve ${args.path} from ${from}`);
      } else target = resolver.resolve(args.path);
      return { path: target, namespace: "node-file" };
    });
    builder.onLoad({ filter: /.*/, namespace: "node-file" }, async (args) => ({
      contents: await readFile(args.path, "utf8"),
      loader:
        path.extname(args.path).slice(1) === "mjs"
          ? "js"
          : path.extname(args.path).slice(1),
    }));
  },
};
const result = await build({
  absWorkingDir: fileURLToPath(new URL(".", import.meta.url)),
  tsconfigRaw: { compilerOptions: { jsx: "react-jsx" } },
  entryPoints: ["./src/main.tsx"],
  bundle: true,
  minify: true,
  sourcemap: false,
  outfile: "../src/roboforge/web/assets/app.js",
  target: ["es2022"],
  plugins: process.env.ROBOFORGE_NODE_RESOLVE ? [nodeFiles] : [],
  write: false,
  define: { "process.env.NODE_ENV": '"production"' },
  legalComments: "external",
});
for (const file of result.outputFiles) {
  await mkdir(path.dirname(file.path), { recursive: true });
  await writeFile(file.path, file.contents);
}
const indexPath = fileURLToPath(
  new URL("../src/roboforge/web/index.html", import.meta.url),
);
let html = await readFile(indexPath, "utf8");
for (const extension of ["js", "css"]) {
  const file = result.outputFiles.find((output) =>
    output.path.endsWith(`app.${extension}`),
  );
  const hash = createHash("sha256")
    .update(file.contents)
    .digest("hex")
    .slice(0, 12);
  html = html.replace(
    new RegExp(`/assets/app\\.${extension}(?:\\?v=[^"\\s]+)?`, "g"),
    `/assets/app.${extension}?v=${hash}`,
  );
}
await writeFile(indexPath, html);
const resolve = createRequire(import.meta.url);
const reactRoot = path.dirname(resolve.resolve("react/package.json"));
const domRoot = path.dirname(resolve.resolve("react-dom/package.json"));
const schedulerRoot = path.dirname(
  createRequire(path.join(domRoot, "package.json")).resolve(
    "scheduler/package.json",
  ),
);
const notices = await Promise.all(
  [
    ["React", reactRoot],
    ["React DOM", domRoot],
    ["Scheduler", schedulerRoot],
  ].map(
    async ([name, root]) =>
      `${name}\n${await readFile(path.join(root, "LICENSE"), "utf8")}`,
  ),
);
await writeFile(
  "../src/roboforge/web/assets/THIRD_PARTY_NOTICES.txt",
  notices.join("\n\n"),
);

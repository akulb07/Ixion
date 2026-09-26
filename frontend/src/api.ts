export async function request<T>(
  path: string,
  options: RequestInit = {},
): Promise<T> {
  const response = await fetch(path, options);
  const body = await response.json();
  if (!response.ok) {
    const detail = body.detail;
    throw new Error(
      Array.isArray(detail)
        ? detail
            .map(
              (e: { loc: string[]; msg: string }) =>
                `${e.loc.slice(1).join(".")}: ${e.msg}`,
            )
            .join("\n")
        : detail || `Request failed (${response.status})`,
    );
  }
  return body;
}
export const post = <T>(path: string, body?: unknown) =>
  request<T>(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
export const ready = (status?: string) =>
  status === "completed" || status === "collision";
export const active = (status?: string) =>
  status === "queued" || status === "running";

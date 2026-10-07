/* A trained rsl_rl actor in the browser: observation normaliser + MLP with ELU, weights exported from the project's
 * TorchScript policy (float32, base64). Same math as the exported policy.pt / policy.onnx:
 *     y = MLP((x - mean) / (std + eps)) */
(function (root) {
  const floats = (b64) => {
    const bin = atob(b64);
    const bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    return new Float32Array(bytes.buffer);
  };

  class Policy {
    constructor(data) {
      this.norm = data.normalizer && { mean: floats(data.normalizer.mean), std: floats(data.normalizer.std), eps: data.normalizer.eps };
      this.layers = data.layers.map((l) => ({ n: l.in, m: l.out, W: floats(l.W), b: floats(l.b) }));
      this.inputSize = this.layers[0].n;
    }

    act(obs) {
      let x = Float64Array.from(obs);
      if (this.norm) for (let i = 0; i < x.length; i++) x[i] = (x[i] - this.norm.mean[i]) / (this.norm.std[i] + this.norm.eps);
      this.layers.forEach(({ n, m, W, b }, k) => {
        const y = new Float64Array(m);
        for (let r = 0; r < m; r++) {
          let s = b[r];
          for (let c = 0, o = r * n; c < n; c++) s += W[o + c] * x[c];
          y[r] = k < this.layers.length - 1 && s < 0 ? Math.expm1(s) : s; // ELU on hidden layers
        }
        x = y;
      });
      return Array.from(x);
    }
  }

  root.Policy = Policy;
})(typeof window !== "undefined" ? window : globalThis);

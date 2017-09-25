MathJax.Hub.Config({
	"HTML-CSS": {
		preferredFont: "STIX"
	},
	TeX: {
		Macros: {
			matr: ["{\\mathbfit #1}", 1],
			vec: ["{\\mathbfit #1}", 1]
		},
	}
});
MathJax.Hub.Register.StartupHook("TeX Jax Ready", function () {
	var MML = MathJax.ElementJax.mml;
	MathJax.InputJax.TeX.Definitions.Add({
		macros: {
			bfit: ['SetFont', MML.VARIANT.BOLDITALIC],
			mathbfit: ['Macro', '{\\bfit #1}', 1]
		}
	});
});

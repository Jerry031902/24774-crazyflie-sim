# Hongfei horizontal LQR: report and validation evidence

Start with [the integration and run instructions](../../START_HERE_HONGFEI.md).

- [Detailed three-week report (PDF)](Hongfei_LQR_Three_Week_Report.pdf)
- [Editable report (Word)](Hongfei_LQR_Three_Week_Report.docx)
- [CPU comparison plot](results/comparison.png)
- [CPU metrics](results/metrics.json)
- [Design parameters](results/design.json)
- [Verification record, dated November 6, 2026](results/verification.txt)
- [Raw CPU CSV files](results/cpu_raw_csv.zip)

The controller replaces horizontal position/velocity control only. The stock
height, attitude, and rate PID loops remain active. Baseline PID is the default;
`hlqr.enable` selects the horizontal LQR within `stabilizer.controller=1`.

The recorded validation comprises 76 passing tests and six nonlinear CPU runs
using the actual C controllers and mixer. CPU runs use assumed dynamics and
synthetic state feedback; they are not Isaac Sim/PhysX or flight results.
The internal CPU scenario label `drag_mismatch` uses beta=0.5/s, which matches
the design value; the label alone should not be interpreted as a mismatch test.

Pending: teammate PhysX runs, target firmware build/flash, automatic braking
integration, supervised hardware flights, and measured execution timing.
The verification record is historical: its GitHub-upload status describes
November 6, not whether a subsequent commit has been made.

Code base: Hongfei branch commit
`96259a3cf0f0911f121c4336335455ef3be446a3`.
Report date: November 7, 2026 (America/New_York).

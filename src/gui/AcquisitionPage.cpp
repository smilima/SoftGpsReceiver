#include "AcquisitionPage.h"
#include "ui_AcquisitionPage.h"
#include "DataTable.h"
#include "PlotWidget.h"
#include "core/Constants.h"

#include <QCheckBox>
#include <cmath>

using namespace gnss;

namespace gui {

AcquisitionPage::AcquisitionPage(QWidget* parent) : PageBase(parent), ui(new Ui::AcquisitionPage)
{
    ui->setupUi(this);
    ui->topSplitter->setStretchFactor(0, 2);
    ui->topSplitter->setStretchFactor(1, 3);
    ui->mainSplitter->setStretchFactor(0, 2);
    ui->mainSplitter->setStretchFactor(1, 5);
    ui->resultsTable->setColumns({tr("PRN"), tr("Acquired"), tr("Metric"), tr("Peak"), tr("2nd peak"), tr("Noise mean"),
                                  tr("C/N0 (dB-Hz)"), tr("Code phase (samples)"), tr("Code phase (chips)"), tr("Coarse freq (Hz)"),
                                  tr("Fine correction (Hz)"), tr("Carrier freq (Hz)"), tr("Doppler (Hz)"), tr("Code Doppler (chips/s)")});
    ui->paramsTable->setColumns({tr("Parameter"), tr("Value")});
    connect(ui->metricPlot, &PlotWidget::clicked, this, [this](double x, double) { selectPrn(int(std::lround(x))); });
    connect(ui->cn0Plot, &PlotWidget::clicked, this, [this](double x, double) { selectPrn(int(std::lround(x))); });
    connect(ui->resultsTable, &QTableWidget::cellClicked, this, [this](int r, int) {
        const QVariant k = ui->resultsTable->keyOfRow(r);
        if (k.isValid()) showPrn(k.toInt());
    });
    connect(ui->chipsAxisCheck, &QCheckBox::toggled, this, [this] { showPrn(prn_); });
}

AcquisitionPage::~AcquisitionPage()
{
    delete ui;
}
void AcquisitionPage::refresh()
{
    PlotWidget* metric_ = ui->metricPlot;
    PlotWidget* cn0_ = ui->cn0Plot;
    PlotWidget* surface_ = ui->surfacePlot;
    PlotWidget* codeSlice_ = ui->codeSlicePlot;
    PlotWidget* dopSlice_ = ui->dopplerSlicePlot;
    PlotWidget* fine_ = ui->finePlot;
    DataTable* table_ = ui->resultsTable;
    DataTable* params_ = ui->paramsTable;
    metric_->clear();
    cn0_->clear();
    table_->beginFill(0);
    table_->endFill();
    params_->beginFill(0);
    params_->endFill();
    surface_->clear(); codeSlice_->clear(); dopSlice_->clear(); fine_->clear();
    if (!readable(Receiver::StageAcquired)) return;
    const AcquisitionOutput& a = rx_->acq;
    const ReceiverSettings& s = rx_->settings;

    QVector<double> px, py, nx, ny, cx, cy, ux, uy;
    for (const auto& r : a.results) {
        if (r.acquired) { px << r.prn; py << r.metric; cx << r.prn; cy << r.cn0; }
        else { nx << r.prn; ny << r.metric; ux << r.prn; uy << r.cn0; }
    }
    metric_->addSeries(tr("acquired"), px, py, PlotWidget::Bars, QColor(44, 160, 44));
    metric_->addSeries(tr("not acquired"), nx, ny, PlotWidget::Bars, QColor(160, 160, 160));
    metric_->series(0).barWidth = metric_->series(1).barWidth = 0.7;
    metric_->addHLine(s.acqThreshold, QColor(214, 39, 40), tr("threshold %1").arg(s.acqThreshold));
    metric_->setYIncludesZero(true);
    metric_->autoScale();
    cn0_->addSeries(tr("acquired"), cx, cy, PlotWidget::Bars, QColor(31, 119, 180));
    cn0_->addSeries(tr("not acquired (noise)"), ux, uy, PlotWidget::Bars, QColor(190, 190, 190));
    cn0_->series(0).barWidth = cn0_->series(1).barWidth = 0.7;
    cn0_->setFixedYRange(25, 60);
    cn0_->autoScale();

    table_->beginFill(int(a.results.size()));
    for (int i = 0; i < int(a.results.size()); ++i) {
        const AcqResult& r = a.results[size_t(i)];
        const QColor bg = r.acquired ? goodColor() : QColor();
        table_->setNum(i, 0, r.prn, 0, bg);
        table_->setKey(i, r.prn);
        table_->setText(i, 1, r.acquired ? tr("yes") : tr("no"), bg);
        table_->setNum(i, 2, r.metric, 3, bg);
        table_->setSci(i, 3, r.peak, 5);
        table_->setSci(i, 4, r.secondPeak, 5);
        table_->setSci(i, 5, r.noiseMean, 5);
        table_->setNum(i, 6, r.cn0, 1);
        table_->setNum(i, 7, r.codePhase, 0);
        table_->setNum(i, 8, r.codePhaseChips, 3);
        table_->setNum(i, 9, r.coarseFreq, 1);
        table_->setNum(i, 10, r.fineDelta, 2);
        table_->setNum(i, 11, r.fineFreq, 2);
        table_->setNum(i, 12, r.doppler, 2);
        table_->setNum(i, 13, r.doppler / kCarrierToCode, 4);
    }
    table_->endFill();

    params_->beginFill(0);
    auto row = [&](const QString& k, const QString& v) {
        const int r = params_->rowCount();
        params_->setRowCount(r + 1);
        params_->setText(r, 0, k);
        params_->setText(r, 1, v);
    };
    row(tr("Method"), tr("Parallel code-phase search (FFT circular correlation)"));
    row(tr("Data start sample"), QString::number(a.startSample));
    row(tr("Coherent integration"), tr("1 ms (%1 samples)").arg(a.samplesPerCode));
    row(tr("Non-coherent blocks"), QString::number(a.nonCoherent));
    row(tr("FFT length / internal"), tr("%1 / %2 (Bluestein for non-power-of-2)").arg(a.fftSize).arg(a.fftInternalSize));
    row(tr("Doppler bins"), tr("%1 bins of %2 Hz, %3 .. %4 Hz").arg(a.binFreqs.size()).arg(s.acqStep)
                                .arg(a.binFreqs.empty() ? 0 : a.binFreqs.front() - s.ifFreq)
                                .arg(a.binFreqs.empty() ? 0 : a.binFreqs.back() - s.ifFreq));
    row(tr("Threshold"), QString::number(s.acqThreshold));
    row(tr("Fine frequency"), tr("%1 ms, squared 0.5-ms correlations (bit-transition insensitive)").arg(s.fineFreqMs));
    row(tr("Processing time"), tr("%1 s").arg(a.elapsedSec, 0, 'f', 2));
    params_->endFill();

    int best = 0;
    double bm = -1;
    for (const auto& r : a.results)
        if (r.metric > bm) { bm = r.metric; best = r.prn; }
    showPrn(prn_ > 0 ? prn_ : best);
}

void AcquisitionPage::selectPrn(int prn)
{
    DataTable* table_ = ui->resultsTable;
    showPrn(prn);
    const int r = table_->rowOfKey(prn);
    if (r >= 0) table_->selectRow(r);
}

void AcquisitionPage::showPrn(int prn)
{
    PlotWidget* surface_ = ui->surfacePlot;
    PlotWidget* codeSlice_ = ui->codeSlicePlot;
    PlotWidget* dopSlice_ = ui->dopplerSlicePlot;
    PlotWidget* fine_ = ui->finePlot;
    if (!readable(Receiver::StageAcquired)) return;
    const AcquisitionOutput& a = rx_->acq;
    const AcqResult* r = nullptr;
    for (const auto& x : a.results)
        if (x.prn == prn) r = &x;
    if (!r || a.binFreqs.empty()) return;
    prn_ = prn;
    const ReceiverSettings& s = rx_->settings;
    const int N = a.samplesPerCode;
    const bool chips = ui->chipsAxisCheck->isChecked();
    const double xmax = chips ? kCaCodeLength : N;
    const double nb = double(a.binFreqs.size());
    const double d0 = a.binFreqs.front() - s.ifFreq - s.acqStep / 2, d1 = a.binFreqs.back() - s.ifFreq + s.acqStep / 2;

    surface_->clear();
    surface_->setLabels(tr("PRN %1 correlation surface |R(tau, fd)|^2 (%2 x %3 cells, max-pooled)")
                            .arg(prn).arg(r->surfaceCols).arg(int(nb)),
                        chips ? tr("Code phase (chips)") : tr("Code phase (samples)"), tr("Doppler (Hz)"));
    surface_->setHeatmap(QVector<float>(r->surface.begin(), r->surface.end()), r->surfaceCols, int(nb), 0, xmax, d0, d1, tr("log10 power"), true);

    const double scale = chips ? double(kCaCodeLength) / N : 1.0;
    codeSlice_->clear();
    codeSlice_->setLabels(tr("Code-phase slice at Doppler %1 Hz (zoom in to see the correlation triangle)")
                              .arg(a.binFreqs[size_t(r->peakBin)] - s.ifFreq),
                          chips ? tr("Code phase (chips)") : tr("Code phase (samples)"), tr("Correlation power"));
    codeSlice_->addSeries(tr("|R|^2"), range(N, scale), toQ(r->codeSlice), PlotWidget::Lines, QColor(31, 119, 180), 1.0);
    codeSlice_->addHLine(r->secondPeak, QColor(255, 127, 14), tr("second peak"), Qt::DotLine);
    codeSlice_->addHLine(r->noiseMean, QColor(127, 127, 127), tr("noise mean"), Qt::DotLine);
    codeSlice_->addVLine(r->codePhase * scale, QColor(214, 39, 40), tr("peak %1").arg(r->codePhase * scale, 0, 'f', 2));

    QVector<double> dx, dy;
    for (size_t b = 0; b < a.binFreqs.size(); ++b) { dx << a.binFreqs[b] - s.ifFreq; dy << r->dopplerSlice[b]; }
    dopSlice_->clear();
    dopSlice_->setLabels(tr("Doppler slice at code phase %1").arg(r->codePhase), tr("Doppler (Hz, relative to nominal IF)"),
                         tr("Correlation power"));
    dopSlice_->addSeries(tr("|R|^2"), dx, dy, PlotWidget::LinesPoints, QColor(44, 160, 44), 1.2);
    dopSlice_->addVLine(r->doppler, QColor(214, 39, 40), tr("fine %1 Hz").arg(r->doppler, 0, 'f', 1));

    fine_->clear();
    fine_->setLabels(tr("Fine frequency search: spectrum of squared 0.5-ms correlations"),
                     tr("Frequency offset from coarse bin (Hz)"), tr("Magnitude"));
    if (!r->fineAxis.empty()) {
        fine_->addSeries(tr("|DFT(P^2)|"), toQ(r->fineAxis), toQ(r->fineSpectrum), PlotWidget::Lines, QColor(148, 103, 189), 1.2);
        fine_->addVLine(r->fineDelta, QColor(214, 39, 40), tr("%1 Hz").arg(r->fineDelta, 0, 'f', 1));
        fine_->addSpan(-s.acqStep / 2, s.acqStep / 2, QColor(44, 160, 44, 30));
    }
}

} // namespace gui

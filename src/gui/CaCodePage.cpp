#include "CaCodePage.h"
#include "ui_CaCodePage.h"
#include "DataTable.h"
#include "PlotWidget.h"
#include "core/CaCode.h"
#include "core/Constants.h"
#include "core/Fft.h"

#include <cmath>

using namespace gnss;

namespace gui {

CaCodePage::CaCodePage(QWidget* parent) : PageBase(parent), ui(new Ui::CaCodePage)
{
    ui->setupUi(this);
    ui->splitter->setStretchFactor(0, 3);
    ui->splitter->setStretchFactor(1, 2);
    ui->codeTable->setColumns({tr("PRN"), tr("G2 taps"), tr("G2 delay (chips)"), tr("First 10 chips (octal)"), tr("ICD octal"),
                               tr("ICD check"), tr("Tap = delay method"), tr("Ones"), tr("Zeros"),
                               tr("Max |autocorr sidelobe|"), tr("Max |cross-corr|"), tr("Worst partner")});
    connect(ui->codeTable, &QTableWidget::cellClicked, this, [this](int r, int) {
        const QVariant k = ui->codeTable->keyOfRow(r);
        if (k.isValid()) ui->prnASpin->setValue(k.toInt());
    });
    connect(ui->prnASpin, &QSpinBox::valueChanged, this, &CaCodePage::updatePlots);
    connect(ui->prnBSpin, &QSpinBox::valueChanged, this, &CaCodePage::updatePlots);
    connect(ui->chipsSpin, &QSpinBox::valueChanged, this, &CaCodePage::updatePlots);
}

CaCodePage::~CaCodePage()
{
    delete ui;
}
void CaCodePage::refresh()
{
    if (!rx_) return;
    if (!tableBuilt_) buildTable();
    updatePlots();
}

void CaCodePage::buildTable()
{
    DataTable* table_ = ui->codeTable;
    tableBuilt_ = true;
    // FFT-based circular correlations of all code pairs
    Fft fft(kCaCodeLength);
    std::vector<std::vector<cplx>> spec(33);
    for (int p = 1; p <= 32; ++p) {
        const auto c = CaCode::chips(p);
        std::vector<cplx> x(kCaCodeLength);
        for (int i = 0; i < kCaCodeLength; ++i) x[size_t(i)] = cplx(c[size_t(i)], 0);
        fft.forward(x);
        spec[size_t(p)] = x;
    }
    int maxAuto[33] = {}, maxCross[33] = {}, worst[33] = {};
    std::vector<cplx> y(kCaCodeLength);
    for (int a = 1; a <= 32; ++a) {
        for (int b = a; b <= 32; ++b) {
            for (int i = 0; i < kCaCodeLength; ++i) y[size_t(i)] = cmul(spec[size_t(a)][size_t(i)], std::conj(spec[size_t(b)][size_t(i)]));
            fft.inverse(y);
            int m = 0;
            for (int i = (a == b ? 1 : 0); i < kCaCodeLength; ++i) m = std::max(m, int(std::lround(std::fabs(y[size_t(i)].real()))));
            if (a == b) { maxAuto[a] = m; continue; }
            if (m > maxCross[a]) { maxCross[a] = m; worst[a] = b; }
            if (m > maxCross[b]) { maxCross[b] = m; worst[b] = a; }
        }
    }
    table_->beginFill(32);
    for (int p = 1; p <= 32; ++p) {
        const int r = p - 1;
        const CaCodeInfo& ci = CaCode::info(p);
        const auto bits = CaCode::bits(p);
        const auto bits2 = CaCode::bitsByDelay(p);
        const int oct = CaCode::firstTenChipsOctal(bits);
        int ones = 0;
        for (auto b : bits) ones += b;
        table_->setNum(r, 0, p, 0);
        table_->setKey(r, p);
        table_->setText(r, 1, QStringLiteral("%1 ⊕ %2").arg(ci.tap1).arg(ci.tap2));
        table_->setNum(r, 2, ci.g2Delay, 0);
        table_->setText(r, 3, QString::number(oct, 8));
        table_->setText(r, 4, QString::number(ci.icdOctal, 8));
        table_->setText(r, 5, oct == ci.icdOctal ? tr("match") : tr("MISMATCH"), oct == ci.icdOctal ? goodColor() : badColor());
        table_->setText(r, 6, bits == bits2 ? tr("identical") : tr("DIFFERENT"), bits == bits2 ? goodColor() : badColor());
        table_->setNum(r, 7, ones, 0);
        table_->setNum(r, 8, kCaCodeLength - ones, 0);
        table_->setNum(r, 9, maxAuto[p], 0);
        table_->setNum(r, 10, maxCross[p], 0);
        table_->setNum(r, 11, worst[p], 0);
    }
    table_->endFill();
}

void CaCodePage::updatePlots()
{
    if (!rx_) return;
    PlotWidget* seq_ = ui->sequencePlot;
    PlotWidget* auto_ = ui->autoPlot;
    PlotWidget* cross_ = ui->crossPlot;
    PlotWidget* tri_ = ui->trianglePlot;
    PlotWidget* spec_ = ui->spectrumPlot;
    const int a = ui->prnASpin->value(), b = ui->prnBSpin->value();
    const int n = ui->chipsSpin->value();
    const auto ca = CaCode::chips(a);
    const auto cb = CaCode::chips(b);
    const auto g1 = CaCode::g1Sequence();
    const auto bitsA = CaCode::bits(a);

    // sequences
    seq_->clear();
    seq_->setLabels(tr("PRN %1: first %2 chips (logic 0 -> +1, 1 -> -1)").arg(a).arg(n), tr("Chip"), QString());
    QVector<double> x(n), yc(n), y1(n), y2(n);
    for (int i = 0; i < n; ++i) {
        x[i] = i;
        yc[i] = ca[size_t(i)];
        y1[i] = 3.0 + (g1[size_t(i)] ? -0.8 : 0.8);
        y2[i] = 6.0 + ((g1[size_t(i)] ^ bitsA[size_t(i)]) ? -0.8 : 0.8);
    }
    seq_->addSeries(tr("C/A code"), x, yc, PlotWidget::Steps, QColor(31, 119, 180), 1.5);
    seq_->addSeries(tr("G1 (offset +3)"), x, y1, PlotWidget::Steps, QColor(44, 160, 44), 1.2);
    seq_->addSeries(tr("G2i (offset +6)"), x, y2, PlotWidget::Steps, QColor(214, 39, 40), 1.2);

    // autocorrelation
    const auto ac = CaCode::circularCorrelation(ca, ca);
    QVector<double> lx, ly;
    for (int k = -511; k <= 511; ++k) { lx << k; ly << ac[size_t((k + kCaCodeLength) % kCaCodeLength)]; }
    auto_->clear();
    auto_->setLabels(tr("Autocorrelation PRN %1 (three-valued: 1023, -1, -65, +63)").arg(a), tr("Lag (chips)"), tr("Correlation"));
    auto_->addSeries(tr("R(k)"), lx, ly, PlotWidget::Sticks, QColor(31, 119, 180), 1.0);
    auto_->addHLine(63, QColor(214, 39, 40), "63", Qt::DotLine);
    auto_->addHLine(-65, QColor(214, 39, 40), "-65", Qt::DotLine);

    const auto cc = CaCode::circularCorrelation(ca, cb);
    QVector<double> cx, cy;
    for (int k = 0; k < kCaCodeLength; ++k) { cx << k; cy << cc[size_t(k)]; }
    cross_->clear();
    cross_->setLabels(tr("Cross-correlation PRN %1 x PRN %2").arg(a).arg(b), tr("Lag (chips)"), tr("Correlation"));
    cross_->addSeries(tr("R_ab(k)"), cx, cy, PlotWidget::Sticks, QColor(255, 127, 14), 1.0);
    cross_->addHLine(63, QColor(214, 39, 40), "63", Qt::DotLine);
    cross_->addHLine(-65, QColor(214, 39, 40), "-65", Qt::DotLine);

    // correlation triangle at the receiver sampling rate
    const double fs = rx_->settings.samplingFreq;
    const int N = rx_->settings.samplesPerCode();
    std::vector<float> s(static_cast<size_t>(N));
    for (int i = 0; i < N; ++i) s[size_t(i)] = ca[size_t(int(std::floor(i * kCaChipRate / fs)) % kCaCodeLength)];
    const double spc = fs / kCaChipRate;
    const int maxLag = int(std::ceil(2.0 * spc));
    QVector<double> tx, ty;
    for (int k = -maxLag; k <= maxLag; ++k) {
        double acc = 0;
        for (int i = 0; i < N; ++i) acc += s[size_t(i)] * s[size_t((i + k + N) % N)];
        tx << k / spc;
        ty << acc / N;
    }
    tri_->clear();
    tri_->setLabels(tr("Correlation peak sampled at %1 MHz (%2 samples/chip)").arg(fs / 1e6, 0, 'f', 3).arg(spc, 0, 'f', 2),
                    tr("Offset (chips)"), tr("Normalised correlation"));
    tri_->addSeries(tr("sampled"), tx, ty, PlotWidget::LinesPoints, QColor(148, 103, 189), 1.2);
    tri_->series(0).pointSize = 2;
    QVector<double> ix, iy;
    for (double d = -2; d <= 2.0001; d += 0.01) { ix << d; iy << (std::fabs(d) < 1 ? 1 - std::fabs(d) * (1 + 1.0 / 1023) : -1.0 / 1023); }
    tri_->addSeries(tr("ideal triangle"), ix, iy, PlotWidget::Lines, QColor(127, 127, 127), 1.0);
    tri_->addVLine(-rx_->settings.dllSpacing, QColor(44, 160, 44), tr("L"), Qt::DotLine);
    tri_->addVLine(rx_->settings.dllSpacing, QColor(44, 160, 44), tr("E"), Qt::DotLine);

    // spectrum of the sampled code: 1 kHz line spectrum under a sinc^2 envelope
    Fft f(N);
    std::vector<cplx> X(static_cast<size_t>(N));
    for (int i = 0; i < N; ++i) X[size_t(i)] = cplx(s[size_t(i)], 0);
    f.forward(X);
    QVector<double> fx, fy, ex, ey;
    const int kmax = std::min(N / 2, int(3.0e6 / (fs / N)));
    for (int k = -kmax; k <= kmax; ++k) {
        const double fr = k * fs / N;
        fx << fr / 1e6;
        const double p = std::norm(X[size_t((k + N) % N)]) / (double(N) * N);
        fy << 10 * std::log10(std::max(p, 1e-12));
    }
    for (double fr = -3e6; fr <= 3e6; fr += 5e3) {
        const double xx = kPi * fr / kCaChipRate;
        const double sinc = std::fabs(xx) < 1e-12 ? 1.0 : std::sin(xx) / xx;
        ex << fr / 1e6;
        ey << 10 * std::log10(std::max(sinc * sinc / 1023.0, 1e-12));
    }
    spec_->clear();
    spec_->setLabels(tr("Power spectrum of PRN %1 (lines every 1 kHz, sinc^2 envelope, nulls at multiples of 1.023 MHz)").arg(a),
                     tr("Frequency offset (MHz)"), tr("Power (dB)"));
    spec_->addSeries(tr("|FFT|^2 of one code period"), fx, fy, PlotWidget::Points, QColor(31, 119, 180), 1.0);
    spec_->series(0).pointSize = 2.5;
    spec_->addSeries(tr("sinc^2 envelope / 1023"), ex, ey, PlotWidget::Lines, QColor(214, 39, 40), 1.5);
}

} // namespace gui

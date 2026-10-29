# Software GPS Receiver (Qt)

A post-processing GPS L1 C/A receiver for recorded IF sample files. It is built to show every step of the
signal processing: raw-signal statistics, C/A code generation, acquisition, tracking loops, bit
synchronisation, navigation-message decoding with parity, ephemeris/almanac/health, satellite orbits,
pseudoranges, and the least-squares position/velocity/time solution.

![Position page](docs/screenshots/17_Position_Summary__plots.png)

## Building and running in Qt Creator

1. Open `CMakeLists.txt` in Qt Creator (*File > Open File or Project*).
2. Select the **Desktop Qt 6.12.0 MinGW 64-bit** kit.
3. Choose the **Release** build configuration and press **Run** (Ctrl+R).
   Debug builds also work: the DSP core (`gnsscore`) is optimised even in Debug
   (option `GNSS_OPTIMIZE_CORE_IN_DEBUG`), so only the GUI is unoptimised.
4. On first start the program looks for a large `*.bin` file in the working directory and up to five levels above
   the executable, so `GPSdata-DiscreteComponents-fs38_192-if9_55.bin` is opened automatically. Otherwise use
   *File > Open*. After that the last file is reopened.
5. Press **F5** (*Run all*), or run the stages one at a time with F6 (acquire), F7 (track), F8 (decode) and F9 (solve).

The sampling frequency, IF and sample format come from the file name (`fs38_192` = 38.192 MHz,
`if9_55` = 9.55 MHz, int8 real samples by default). You can change them in *File > Settings*, along with every
acquisition, tracking-loop and navigation parameter.

The file name gives the IF rounded to 9.55 MHz. The true IF of this recording is fs/4 = 9.548 MHz. The receiver
copes either way, and the Position page reports the measured *effective IF* (9.547984 MHz) from the
Doppler/clock-drift solution. If you set the IF to 9.548 MHz, the Doppler values are shown without the
2 kHz offset.

## Editing the GUI in Qt Designer

Every page, the main window and the settings dialog are `.ui` forms in `src/gui/`:

| Form | Content |
|---|---|
| `MainWindow.ui` | menus, toolbar, log dock, and the tab widget holding all pages (each tab is a promoted page widget) |
| `SettingsDialog.ui` | input, acquisition, tracking and navigation settings |
| `OverviewPage.ui` | recording info, pipeline status, satellite summary, position summary, algorithm documentation |
| `RawSignalPage.ui` | time-domain samples, ADC histogram, Welch PSD, spectrogram, statistics |
| `CaCodePage.ui` | G1/G2/C-A sequences, autocorrelation, cross-correlation, sampled correlation peak, code spectrum, code table |
| `AcquisitionPage.ui` | metric and C/N0 bars, results table, correlation surface heat map, code/Doppler slices, fine-frequency spectrum |
| `TrackingPage.ui` | 12 per-channel loop plots, all-channel overlays, multi-correlator shape, loop design tables |
| `BitSyncPage.ui` | transition histogram, prompt with bit edges, soft bits, preamble correlation, raw bit grid |
| `NavMessagePage.ui` | subframe list, words with parity check, colour-coded bit map, decoded fields with scaling and interpretation |
| `EphemerisPage.ui` | ephemeris table, almanac, health/A-S/configuration, iono/UTC, special message, NMCT, received pages |
| `SatellitesPage.ui` | sky plot, elevation, sub-satellite points, satellite state, step-by-step Kepler computation |
| `MeasurementsPage.ui` | per-epoch pseudorange corrections table, PR change, residuals, code-minus-carrier, Doppler, tropo, SV clock |
| `PositionPage.ui` | summary, scatter/ENU/clock/DOP/velocity plots, solutions table, least-squares iterations, H and Q matrices |

Notes for editing:

* The plots and tables are **promoted widgets**: `gui::PlotWidget` and `gui::SkyPlotWidget` (from `QWidget`) and
  `gui::DataTable` (from `QTableWidget`). You can move, resize, split or regroup them freely.
* Plot titles and axis labels are dynamic properties of the promoted widget (`title`, `xLabel`, `yLabel`, plus
  `aspectLocked` and `legendVisible`). Edit them in the Property Editor. Some titles are replaced at run time
  with PRN-specific text.
* Keep the **objectName** of any control the code uses (e.g. `channelCombo`, `epochSpin`, `surfacePlot`). The code
  accesses them as `ui->name`.
* Table **column headers are set in code** (`setColumns(...)` in each page's constructor), because the code fills
  the table by column index. Change the header list and the matching `setNum/setText` column numbers together.
* The combo boxes in `SettingsDialog.ui` list their items in enum order (sample format, Doppler sign). Keep that order.

Plot interaction: mouse wheel zooms (Ctrl = x only, Shift = y only), left-drag box-zooms, right-drag pans,
double-click resets, and right-click opens a menu (copy/save image, export CSV, legend, grid). The tracking-page
time axes are linked. Tables support Ctrl+C and *Export to CSV* from the context menu.

## Command line

```
gnss_cli <file.bin> [--ms N] [--channels N] [--fs Hz] [--if Hz] [--csv solutions.csv]
SoftGpsReceiver <file.bin> --screenshots <dir>   # process, save a PNG of every page and sub-tab, exit
```

## Results for the sample recording

`GPSdata-DiscreteComponents-fs38_192-if9_55.bin` is 50.08 s of 4-bit-quality int8 real samples.

* 9 satellites acquired (PRN 3, 6, 9, 15, 18, 21, 22, 26, 29). All tracked with 100 % phase lock at 42–51 dB-Hz.
* 63 subframes decoded, 630/630 words pass parity, and all 9 ephemerides complete and consistent.
* 99 fixes (every 0.5 s): **40.00802° N, 105.26270° W, h = 1634 m** (Boulder, Colorado), with σ E/N/U =
  1.95/2.08/3.27 m and PDOP 1.65.
* GPS week 297 (mod 1024), i.e. 2005-05-07 19:10 UTC.
* Sample clock error +10.4 ppb (clock-bias slope 3.1 m/s). Effective IF 9.547984 MHz.
* Subframe 4 page 18 (ionosphere/UTC) is broadcast every 12.5 min and is not in this 50-s recording, so no
  ionospheric correction is applied (residual RMS ≈ 5 m).

Processing the whole file takes about 13 s on a 12-core CPU: acquisition 2 s, tracking 9 s with one thread per channel.

## Screenshots

All screenshots below come from processing the sample recording. You can regenerate them with
`SoftGpsReceiver <file.bin> --screenshots <dir>`.

### Overview
Recording parameters, pipeline status, a per-satellite summary (acquisition, tracking, bit sync, parity,
ephemeris, health, elevation) and the built-in description of the algorithms.

![Overview](docs/screenshots/01_Overview.png)

### Raw signal
Time-domain samples, ADC level histogram against a Gaussian, Welch power spectral density showing the
front-end passband around the IF, spectrogram, and signal statistics.

![Raw signal](docs/screenshots/02_Raw_signal.png)

### C/A codes
G1, G2<sub>i</sub> and C/A chip sequences, the three-valued autocorrelation, cross-correlation between two PRNs,
the correlation peak sampled at the receiver rate, the 1 kHz line spectrum, and a table that checks all 32 codes
against IS-GPS-200.

![C/A codes](docs/screenshots/03_C-A_codes.png)

### Acquisition
Detection metric and C/N0 for every PRN, the full results table, and for the selected PRN: the code-phase ×
Doppler correlation surface, code-phase and Doppler slices through the peak, and the fine-frequency search.

![Acquisition](docs/screenshots/04_Acquisition.png)

### Tracking
**Channel detail:** prompt I/Q scatter, prompt correlator outputs (navigation bits), early/prompt/late
envelopes, PLL, FLL and DLL discriminators, carrier Doppler, code NCO, C/N0, phase lock indicator, accumulated
Doppler and code-epoch drift, all on linked time axes.

![Tracking - channel detail](docs/screenshots/05_Tracking_Channel_detail.png)

**All channels:** C/N0, Doppler, phase lock indicator and DLL error overlaid for every channel.

![Tracking - all channels](docs/screenshots/06_Tracking_All_channels.png)

**Correlator shape:** the correlation function recomputed with a bank of correlators from the stored NCO
state, showing the peak rounded by the front-end filter, with the loop's early/prompt/late points on it.

![Tracking - correlator shape](docs/screenshots/07_Tracking_Correlator_shape.png)

**Summary & loop design:** per-channel statistics and the PLL/FLL/DLL loop-filter coefficients.

![Tracking - summary and loop design](docs/screenshots/08_Tracking_Summary__loop_design.png)

### Bit sync & data bits
Histogram of prompt sign transitions modulo 20 ms, prompt I with the detected bit edges, the soft bits and their
distribution, the bit stream correlated with the preamble (parity-verified subframe starts in red), and the raw
bit grid.

![Bit sync and data bits](docs/screenshots/09_Bit_sync__data_bits.png)

### Navigation message
Every decoded subframe (TOW, page, parity, polarity, flags). For the selected subframe: each word with its
received and computed parity, a colour-coded map of the 300 bits by field, and every field with raw bits, scale
factor, value, units and interpretation.

![Navigation message](docs/screenshots/10_Navigation_message.png)

### Ephemeris, almanac & health
Broadcast ephemeris and clock parameters for all satellites, decoded almanac pages, satellite health from
subframe 1, the almanac and the page-25 summaries, and the ionosphere/UTC, special-message, NMCT and
received-pages tables.

![Ephemeris](docs/screenshots/11_Ephemeris,_almanac__health_Ephemeris_%28subframes_1-3%29.png)

![Almanac](docs/screenshots/12_Ephemeris,_almanac__health_Almanac.png)

![Health and configuration](docs/screenshots/13_Ephemeris,_almanac__health_Health__configuration.png)

![Iono, UTC and other pages](docs/screenshots/14_Ephemeris,_almanac__health_Iono_-_UTC__other_pages.png)

### Satellites & orbits
Sky plot with tracks, elevation over time, sub-satellite points, satellite position/velocity/clock at the
selected epoch, and the IS-GPS-200 orbit computation step by step (including each Kepler iteration).

![Satellites and orbits](docs/screenshots/15_Satellites__orbits.png)

### Pseudoranges
Every correction applied to each pseudorange at the selected epoch, with pseudorange change, post-fit residuals,
code-minus-carrier, measured vs predicted Doppler, tropospheric delay and satellite clock correction over time.

![Pseudoranges](docs/screenshots/16_Pseudoranges.png)

### Position
**Summary & plots:** mean position, accuracy statistics, horizontal scatter with CEP/2DRMS circles, E/N/U
error, receiver clock bias and drift, DOP, satellites used, and Doppler velocity.

![Position - summary and plots](docs/screenshots/17_Position_Summary__plots.png)

**Solutions:** every epoch's fix (geodetic, ECEF, ENU, clock, DOP, velocity, residual RMS), exportable to CSV/KML.

![Position - solutions](docs/screenshots/18_Position_Solutions.png)

**Least-squares detail:** every Gauss–Newton iteration from the Earth's centre to convergence, the design
matrix and residuals, and the cofactor matrix in the local E/N/U/clock frame.

![Position - least-squares detail](docs/screenshots/19_Position_Least-squares_detail.png)

## Code structure

```
src/core/            GUI-independent receiver (Qt Core + Concurrent only)
  Settings           parameters, file-name parsing, persistence
  SignalSource       memory-mapped sample file (int8/uint8/int16, real or I/Q)
  Fft                radix-2 + Bluestein FFT (any length)
  CaCode             G1/G2 Gold code generator (tap and delay methods), ICD table
  SignalAnalysis     statistics, Welch PSD, spectrogram
  Acquisition        parallel code-phase search, fine frequency estimation
  Tracking           FLL-assisted Costas PLL + early/late DLL, C/N0, lock detector, multi-correlator
  NavMessage         bit sync, preamble search, parity, IS-GPS-200 field layouts, ephemeris/almanac/health/iono
  Ephemeris          satellite position/velocity/clock (Table 20-IV) with step-by-step trace
  Geodesy            ECEF/geodetic/ENU/UTM, az/el, Saastamoinen, Klobuchar, GPS time
  Pvt                pseudoranges, corrections, iterative least squares, DOP, Doppler velocity
  Receiver           pipeline orchestration (runs on a worker thread)
src/gui/             Qt Widgets front end (.ui forms + page classes, PlotWidget, SkyPlotWidget, DataTable)
src/cli/             command-line front end
docs/algorithms.html algorithm description shown on the Overview page
```

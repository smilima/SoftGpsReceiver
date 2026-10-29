#include "DataTable.h"

#include <QApplication>
#include <QClipboard>
#include <QContextMenuEvent>
#include <QFile>
#include <QFileDialog>
#include <QHeaderView>
#include <QKeyEvent>
#include <QMenu>
#include <QTextStream>
#include <cmath>

namespace gui {

namespace {

constexpr int kSortRole = Qt::UserRole + 10;
constexpr int kKeyRole = Qt::UserRole + 11;

class SortItem : public QTableWidgetItem
{
public:
    using QTableWidgetItem::QTableWidgetItem;
    bool operator<(const QTableWidgetItem& o) const override
    {
        const QVariant a = data(kSortRole), b = o.data(kSortRole);
        if (a.isValid() && b.isValid()) return a.toDouble() < b.toDouble();
        return text() < o.text();
    }
};

} // namespace

DataTable::DataTable(QWidget* parent) : QTableWidget(parent)
{
    setAlternatingRowColors(true);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setEditTriggers(QAbstractItemView::NoEditTriggers);
    verticalHeader()->setDefaultSectionSize(fontMetrics().height() + 6);
    verticalHeader()->setVisible(false);
    horizontalHeader()->setHighlightSections(false);
    horizontalHeader()->setSectionsMovable(true);
    setWordWrap(false);
}

void DataTable::setColumns(const QStringList& headers)
{
    setColumnCount(int(headers.size()));
    setHorizontalHeaderLabels(headers);
}

void DataTable::beginFill(int rows)
{
    setSortingEnabled(false);
    clearContents();
    setRowCount(rows);
}

void DataTable::endFill(bool resizeColumns)
{
    if (resizeColumns) resizeColumnsToContents();
    setSortingEnabled(sortable_);
}

QTableWidgetItem* DataTable::item_(int r, int c)
{
    QTableWidgetItem* it = item(r, c);
    if (!it) {
        it = new SortItem();
        setItem(r, c, it);
    }
    return it;
}

void DataTable::setText(int r, int c, const QString& text, const QColor& bg, const QString& tip)
{
    QTableWidgetItem* it = item_(r, c);
    it->setText(text);
    if (bg.isValid()) it->setBackground(bg);
    if (!tip.isEmpty()) it->setToolTip(tip);
}

void DataTable::setNum(int r, int c, double v, int prec, const QColor& bg)
{
    QTableWidgetItem* it = item_(r, c);
    it->setText(std::isfinite(v) ? QString::number(v, 'f', prec) : QStringLiteral("-"));
    it->setData(kSortRole, v);
    it->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    if (bg.isValid()) it->setBackground(bg);
}

void DataTable::setSci(int r, int c, double v, int prec, const QColor& bg)
{
    QTableWidgetItem* it = item_(r, c);
    it->setText(std::isfinite(v) ? QString::number(v, 'g', prec) : QStringLiteral("-"));
    it->setData(kSortRole, v);
    it->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    if (bg.isValid()) it->setBackground(bg);
}

void DataTable::setKey(int r, const QVariant& key)
{
    item_(r, 0)->setData(kKeyRole, key);
}

QVariant DataTable::keyOfRow(int r) const
{
    const QTableWidgetItem* it = item(r, 0);
    return it ? it->data(kKeyRole) : QVariant();
}

QVariant DataTable::currentKey() const
{
    const int r = currentRow();
    return r >= 0 ? keyOfRow(r) : QVariant();
}

int DataTable::rowOfKey(const QVariant& key) const
{
    for (int r = 0; r < rowCount(); ++r)
        if (keyOfRow(r) == key) return r;
    return -1;
}

QString DataTable::toCsv(bool selectionOnly) const
{
    QString out;
    QTextStream ts(&out);
    auto esc = [](QString s) {
        if (s.contains(',') || s.contains('"')) s = '"' + s.replace('"', "\"\"") + '"';
        return s;
    };
    QStringList hdr;
    for (int c = 0; c < columnCount(); ++c)
        hdr << esc(horizontalHeaderItem(c) ? horizontalHeaderItem(c)->text() : QString());
    ts << hdr.join(',') << '\n';
    for (int r = 0; r < rowCount(); ++r) {
        if (selectionOnly) {
            bool sel = false;
            for (int c = 0; c < columnCount() && !sel; ++c)
                if (item(r, c) && item(r, c)->isSelected()) sel = true;
            if (!sel) continue;
        }
        QStringList row;
        for (int c = 0; c < columnCount(); ++c) row << esc(item(r, c) ? item(r, c)->text() : QString());
        ts << row.join(',') << '\n';
    }
    return out;
}

void DataTable::copySelection()
{
    QApplication::clipboard()->setText(toCsv(true).replace(',', '\t'));
}

void DataTable::exportCsv()
{
    const QString fn = QFileDialog::getSaveFileName(this, tr("Export table"), QString(), tr("CSV files (*.csv)"));
    if (fn.isEmpty()) return;
    QFile f(fn);
    if (f.open(QIODevice::WriteOnly | QIODevice::Text)) f.write(toCsv(false).toUtf8());
}

void DataTable::keyPressEvent(QKeyEvent* e)
{
    if (e->matches(QKeySequence::Copy)) { copySelection(); return; }
    QTableWidget::keyPressEvent(e);
}

void DataTable::contextMenuEvent(QContextMenuEvent* e)
{
    QMenu m(this);
    m.addAction(tr("Copy selected rows"), this, &DataTable::copySelection);
    m.addAction(tr("Copy whole table"), this, [this] { QApplication::clipboard()->setText(toCsv(false).replace(',', '\t')); });
    m.addAction(tr("Export to CSV..."), this, &DataTable::exportCsv);
    m.addSeparator();
    m.addAction(tr("Resize columns"), this, [this] { resizeColumnsToContents(); });
    m.exec(e->globalPos());
}

} // namespace gui

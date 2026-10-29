#pragma once

#include <QColor>
#include <QStringList>
#include <QTableWidget>
#include <QVariant>

namespace gui {

// QTableWidget with numeric sorting, copy (Ctrl+C) and CSV export.
class DataTable : public QTableWidget
{
    Q_OBJECT
public:
    explicit DataTable(QWidget* parent = nullptr);

    void setColumns(const QStringList& headers);
    void beginFill(int rows);
    void endFill(bool resizeColumns = true);

    void setText(int r, int c, const QString& text, const QColor& bg = QColor(), const QString& tip = QString());
    void setNum(int r, int c, double v, int prec, const QColor& bg = QColor());
    void setSci(int r, int c, double v, int prec = 8, const QColor& bg = QColor());
    void setKey(int r, const QVariant& key);              // stored on column 0
    QVariant keyOfRow(int r) const;
    QVariant currentKey() const;
    int rowOfKey(const QVariant& key) const;

    QString toCsv(bool selectionOnly) const;

public slots:
    void copySelection();
    void exportCsv();

protected:
    void keyPressEvent(QKeyEvent* e) override;
    void contextMenuEvent(QContextMenuEvent* e) override;

private:
    QTableWidgetItem* item_(int r, int c);
    bool sortable_ = true;
};

} // namespace gui

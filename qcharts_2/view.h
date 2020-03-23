#ifndef VIEW_H
#define VIEW_H
#include <QtWidgets/QGraphicsView>
#include <QtCharts/QChartGlobal>
#include <Qtcharts/QSplineSeries>
#include <QtCharts/QChart>
#include <string>
#include <utility>

#include "daa_datatypes_icd\trackICD.h"

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QMouseEvent;
class QResizeEvent;
QT_END_NAMESPACE

QT_CHARTS_BEGIN_NAMESPACE
class QChart;
QT_CHARTS_END_NAMESPACE

class Callout;

QT_CHARTS_USE_NAMESPACE

const int radarPort=2201;
const int adsbPort=2200;
const int truthPort=2100;

typedef std::pair<QLineSeries, QLineSeries> seriesPair;
typedef std::pair<std::string, std::string> labelPair;

class View: public QGraphicsView{
    Q_OBJECT

public:
    enum class Plot{ alt, nsv, esv, gta, lati, lon, latlon,
                   relran, relalt, relbear};
    View(seriesPair &series, labelPair &&labels,
         Plot plot, bool adsbMode=true,
         QWidget *parent=nullptr);

    QLineSeries *getActualSeries(){return m_actualSeries;}
    QLineSeries *&getTruthSeries(){return m_truthSeries;}
    Plot getPlot(){ return m_plot;}
    void cleanLabels();

protected:
    void resizeEvent(QResizeEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void wheelEvent(QWheelEvent *event);

public slots:
    void keepCallout();
    void tooltip(QPointF point, bool state);
    void handleNewTrack(QByteArray &buffer, int port);
    void rangeChangedX(qreal min, qreal max);
    void rangeChangedY(qreal min, qreal max);
    void connectMarkers();
    void disconnectMarkers();
    void handleMarkerClicked();

signals:
    void enableAdsb(bool enabled);
    void enableRadar(bool enabled);
    void signalData(double x, double y, std::string const &src);

private:
    void adjustRange(double x, double y);
    void relationship(float xx);
    void init();

    QGraphicsSimpleTextItem *m_coordX=nullptr, *m_coordY=nullptr;
    QChart  *m_chart=nullptr;
    Callout *m_tooltip=nullptr, *m_tooltipTruth=nullptr, *m_tooltipActual=nullptr;
    QList<Callout *> m_callouts;
    labelPair m_labels;
    QLineSeries *m_actualSeries, *m_truthSeries;
    Plot m_plot;
    QAbstractAxis *m_axisX, *m_axisY;
    bool m_adsbMode=true; //when false is radar
    double m_maxX=0, m_maxY=0, m_minX=DBL_MAX, m_minY=DBL_MAX;
    double m_deltaX=1, m_deltaY=1;
    bool m_leftButtonPressed= false, m_rightButtonPressed=false, m_wheelPressed=false;
    bool m_radarData=false, m_adsbData=false;
    signed m_mouseX=0, m_mouseY=0;
};

#endif // VIEW_H

#include <QtGui/QResizeEvent>
#include <QtWidgets/QGraphicsScene>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QSplineSeries>
#include <QtWidgets/QGraphicsTextItem>
#include <QtGui/QMouseEvent>
#include <QtCharts/QValueAxis>
#include <QtCharts/QXYLegendMarker>

#include <iostream>
#include "callout.h"
#include "view.h"


View::View(seriesPair &series,  labelPair &&labels, Plot plot, bool adsbMode,
           QWidget *parent)
        : QGraphicsView(new QGraphicsScene, parent),
          m_plot(plot), m_adsbMode(adsbMode),
          m_actualSeries(&series.first), m_truthSeries(&series.second){

    setDragMode(QGraphicsView::NoDrag);
    //setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    //setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_labels = labels;
    // chart
    m_chart = new QChart;
    m_chart->setMinimumSize(820, 600);

    QString aux=labels.first.c_str();
    aux+="-"; aux+=labels.second.c_str(); aux+=" CHART";
    m_chart->setTitle(aux);
    //m_chart->legend()->hide();

    m_chart->addSeries(m_actualSeries);
    m_chart->addSeries(m_truthSeries);

    m_chart->createDefaultAxes();
    m_chart->setAcceptHoverEvents(true);
    m_chart->setAnimationOptions(QChart::AllAnimations);
    m_axisX=m_chart->axisX();
    m_axisY=m_chart->axisY();

    m_axisX->setTitleText(labels.first.c_str());
    m_axisX->setLabelsVisible(true);
    m_axisY->setTitleText(labels.second.c_str());
    m_axisY->setLabelsVisible(true);

    series.second.setName("truth data");
    if (m_adsbMode)
        series.first.setName("actual(adsb) data");
    else
        series.first.setName("actual(radar) data");
    series.first.setPointsVisible(true);
    series.second.setPointsVisible(true);

    setRenderHint(QPainter::Antialiasing);
    scene()->addItem(m_chart);

    m_coordX = new QGraphicsSimpleTextItem(m_chart);
    m_coordX->setPos(m_chart->size().width()/2 - 100, m_chart->size().height());
    m_coordX->setText("X: ");
    m_coordY = new QGraphicsSimpleTextItem(m_chart);
    m_coordY->setPos(m_chart->size().width()/2 + 100, m_chart->size().height());
    m_coordY->setText("Y: ");

    //connect(&series.first, &QLineSeries::clicked, this, &View::keepCallout);
    connect(m_actualSeries, &QLineSeries::pressed, this, &View::keepCallout);
    connect(m_actualSeries, &QLineSeries::hovered, this, &View::tooltip);

    connect(m_truthSeries, &QLineSeries::pressed, this, &View::keepCallout);
    connect(m_truthSeries, &QLineSeries::hovered, this, &View::tooltip);

    QValueAxis *x= qobject_cast<QValueAxis *>(m_axisX);
    connect(x, &QValueAxis::rangeChanged, this, &View::rangeChangedX);
    QValueAxis *y= qobject_cast<QValueAxis *>(m_axisY);
    connect(y, &QValueAxis::rangeChanged, this, &View::rangeChangedY);

    //connect(&series.first, &QLineSeries::doubleClicked, this, &View::doubleClickRelationship);
    connectMarkers();
    this->setMouseTracking(true);
    init();
}


void View::init(){
    m_maxX=350;
    m_minX=0;
    m_deltaX = 10;
    m_deltaY = 1;

    switch(m_plot){
    case Plot::alt:
        m_maxY=3049;
        m_minY=3047;
        m_deltaY = 1;
        break;

    case Plot::nsv:
        m_maxY=77;
        m_minY=74;
        m_deltaY = 1;
        break;

    case Plot::esv:
        m_maxY=-6;
        m_minY=-9;
        m_deltaY = 1;
        break;

    case Plot::gta:
        m_maxY=-4;
        m_minY=-7;
        m_deltaY = 1;
        break;

    case Plot::lati:
        m_maxY=34.0;
        m_minY=33.75;
        m_deltaY = 0.05;
        break;

    case Plot::lon:
        m_maxY=-116.9650;
        m_minY=-116.9930;
        m_deltaY = 0.05;
        break;

    case Plot::latlon:
        m_maxY=-116.9650;
        m_minY=-116.9930;
        m_maxX=34.0;
        m_minX=33.75;
        m_deltaX = 0.05;
        m_deltaY = 0.05;
        break;

    case Plot::relalt:
        m_maxY  =205;
        m_minY  =-460;
        m_deltaY= 10;
        break;

    case Plot::relran:
        m_maxY  =9000;
        m_minY  =0;
        m_deltaY= 200;
        break;

    case Plot::relbear:
        m_maxY  =120;
        m_minY  =0;
        m_deltaY = 10;
        break;

    default:
        ;
    }
    m_axisX->setRange(m_minX, m_maxX);
    m_axisY->setRange(m_minY, m_maxY);
}

void View::resizeEvent(QResizeEvent *event){
    if (scene()) {
        scene()->setSceneRect(QRect(QPoint(0, 0), event->size()));
         m_chart->resize(event->size());
         m_coordX->setPos(m_chart->size().width()/2 - 100, m_chart->size().height() - 20);
         m_coordY->setPos(m_chart->size().width()/2 + 100, m_chart->size().height() - 20);

         const auto callouts = m_callouts;
         for (Callout *callout : callouts)
             callout->updateGeometry();
    }
    QGraphicsView::resizeEvent(event);
}

void View::mouseMoveEvent(QMouseEvent *e){
    QString x(m_labels.first.c_str()), y(m_labels.second.c_str());
    x+=": %1";y+=": %1";
    m_coordX->setText(x.arg(m_chart->mapToValue(e->pos()).x(), 0, 'f', 8));
    m_coordY->setText(y.arg(m_chart->mapToValue(e->pos()).y(), 0, 'f', 8));
    int delta=10;
    bool buttonPressed = m_leftButtonPressed || m_rightButtonPressed;
    if (m_rightButtonPressed) delta = 20;

    if ( buttonPressed ){
        if ( m_mouseX < e->pos().x() )
            m_chart->scroll(+delta, 0);
        else if( m_mouseX > e->pos().x()  )
          m_chart->scroll(-delta, 0);
        if( m_mouseY < e->pos().y() )
          m_chart->scroll(0, +delta);
        else if( m_mouseY > e->pos().y()  )
          m_chart->scroll(0, -delta);
    }

    m_mouseX = e->pos().x();  m_mouseY = e->pos().y();
    QGraphicsView::mouseMoveEvent(e);
}

void View::keepCallout(){
    m_callouts.append(m_tooltip);
    m_tooltip = new Callout(m_chart);
    m_tooltip->setColor(m_actualSeries->color());
}


void View::tooltip(QPointF point, bool state){
    if (m_tooltip == 0) m_tooltip = new Callout(m_chart);

    if (state) {
        QString aux(m_labels.first.c_str());
        aux+=": %1 \n";
        aux+=m_labels.second.c_str();
        aux+=": %2";
        m_tooltip->setText(aux.arg(point.x(), 0, 'f', 8).arg(point.y(), 0, 'f', 8));
        m_tooltip->setAnchor(point);
        m_tooltip->setZValue(11);
        m_tooltip->updateGeometry();
        m_tooltip->show();
    }
    else {
        m_tooltip->hide();
    }
}

void View::wheelEvent(QWheelEvent *event){
    if (event->delta()>0) m_chart->zoomIn();
    else   m_chart->zoomOut();
}

void View::mousePressEvent(QMouseEvent *e){
    if (e->buttons() == Qt::LeftButton)
        m_leftButtonPressed = true;
    if (e->buttons() == Qt::RightButton)
        m_rightButtonPressed = true;
    if(e->buttons() == Qt::MiddleButton )
        m_wheelPressed = true;

    if (m_wheelPressed){
        QPoint qp(m_mouseX, m_mouseY);
        relationship(m_chart->mapToValue(qp).x());
    }

    QGraphicsView::mousePressEvent(e);
}

void View::mouseReleaseEvent(QMouseEvent *e){
    if (e->button() == Qt::LeftButton)
        m_leftButtonPressed = false;
    if (e->button() == Qt::RightButton)
        m_rightButtonPressed = false;
    if (e->button() == Qt::MiddleButton )
        m_wheelPressed = false;

    if (!m_wheelPressed && m_tooltipTruth){
        m_tooltipTruth->hide();
        m_tooltipActual->hide();
    }

    QGraphicsView::mouseReleaseEvent(e);
}

void View::keyPressEvent(QKeyEvent *event){
    switch (event->key()) {
    case Qt::Key_Plus:
        m_chart->zoomIn();
        break;

    case Qt::Key_Minus:
        m_chart->zoomOut();
        break;

    case Qt::Key_Left:
        m_chart->scroll(-10, 0);
        break;

    case Qt::Key_Right:
        m_chart->scroll(+10, 0);
        break;

    case Qt::Key_Up:
        m_chart->scroll(0, +10);
        break;

    case Qt::Key_Down:
        m_chart->scroll(0, -10);
        break;

    case Qt::Key_F6:
        cleanLabels();
        break;

    case Qt::Key_F7:
        std::cout<<"clean actual Serie"<<std::endl;
        m_actualSeries->clear();
        break;

    case Qt::Key_F8:
        std::cout<<"clean truth Serie"<<std::endl;
        m_truthSeries->clear();
        break;

    default:
        QGraphicsView::keyPressEvent(event);
        break;
    }
}

void View::handleNewTrack(QByteArray &buffer, int port){
    trackICD::track track;
    trackICD::track_buffer *ptr=(trackICD::track_buffer*)buffer.data();
    trackICD::read_track(track, *ptr);

    std::cout<<"handleNewTrack("<<port<<"), track time: "<<track.track_time/1000000<<std::endl;

    double x=track.track_time/1000000;
    double y=0;
    if (m_plot == Plot::alt && track.aircraft_state.available_states.GEOMETRIC_ALTITUDE)
        y=track.aircraft_state.geometric_altitude;
    else if (m_plot == Plot::nsv && track.aircraft_state.available_states.GROUND_SPEED_NORTH)
        y=track.aircraft_state.ground_speed_north;
    else if (m_plot == Plot::esv && track.aircraft_state.available_states.GROUND_SPEED_EAST)
        y=track.aircraft_state.ground_speed_east;
    else if (m_plot == Plot::gta && track.aircraft_state.available_states.GROUND_TRACK_ANGLE)
        y=track.aircraft_state.ground_track_angle;
    else if (m_plot == Plot::lati && track.aircraft_state.available_states.LATITUDE)
        y=track.aircraft_state.latitude;
    else if (m_plot == Plot::lon && track.aircraft_state.available_states.LONGITUDE)
        y=track.aircraft_state.longitude;
    else if (m_plot == Plot::latlon &&
             track.aircraft_state.available_states.LONGITUDE &&
             track.aircraft_state.available_states.LATITUDE ){
        y=track.aircraft_state.longitude;
        x=track.aircraft_state.latitude;
    }
    else if (m_plot == Plot::relalt){
        y = track.track_state.relative_altitude;
    }
    else if (m_plot == Plot::relran){
        y = track.track_state.relative_range;
    }
    else if (m_plot == Plot::relbear){
        y = track.track_state.relative_bearing;
    }
    else{
        std::cerr<<"warning: handleNewTrack:: not plot match!"<<std::endl;
        return;
    }

    std::string auxstr;
    if ( track.source_data.ADSB_TRACK && m_adsbMode){
        m_actualSeries->append(x, y);
        std::cout<<"ADSB SRC: new plot data ("<<x<<", "<<y<<")"<<std::endl;
        adjustRange(x, y);
        if (!m_adsbData){
            m_adsbData = true;
            emit enableAdsb(true);
        }
        auxstr="ADSB";
    }
    else if (track.source_data.UNKNOWN_SOURCE || port==truthPort){
        m_truthSeries->append(x, y);
        std::cout<<"TRUTH SRC: new plot data ("<<x<<", "<<y<<")"<<std::endl;
        adjustRange(x, y);
        if (!m_adsbData || !m_radarData){
            m_adsbData = true;
            m_radarData = true;
            emit enableAdsb(true);
            emit enableRadar(true);
        }
        auxstr="TRUTH";
    }
    else if ((track.source_data.RADAR_TRACK && !m_adsbMode)|| port==radarPort){
        m_actualSeries->append(x, y);
        std::cout<<"RADAR SRC: new plot data ("<<x<<", "<<y<<")"<<std::endl;
        adjustRange(x, y);
        if (!m_radarData){
            m_radarData = true;
            emit enableRadar(true);
        }
        auxstr="RADAR";
    }
    else{
        std::cerr<<"warning: HandleNewTrack: ANOTHER SRC, ignored data..."<<std::endl;
        return;
    }
    auxstr+=" ("+m_labels.first+", "+m_labels.second+")";
    emit signalData(x, y, auxstr);
}


void View::adjustRange(double x, double y){
    bool flagX=false, flagY=false;
    
    if (x>m_maxX) {
        m_maxX+=m_deltaX; flagX=true;
    }
    if (x<m_minX) {
        m_minX-=m_deltaX; flagX=true;
    }

    if (y>m_maxY){
        m_maxY+=m_deltaY; flagY=true;
    }
    if (y<m_minY){
        m_minY-=m_deltaY;  flagY=true;
    }

    if (flagX){
        m_axisX->setRange(m_minX, m_maxX);
    }
    if (flagY){
        m_axisY->setRange(m_minY, m_maxY);
     }
}

void View::rangeChangedX(qreal min, qreal max){
    std::cout<<"X RANGE CHANGED -> ("<<min<<", "<<max<<")"<<std::endl;
    m_minX = min; m_maxX = max;
}

void View::rangeChangedY(qreal min, qreal max){
    std::cout<<"Y RANGE CHANGED -> ("<<min<<", "<<max<<")"<<std::endl;
    m_minY = min; m_maxY = max;
}

void View::cleanLabels(){
    foreach(auto T, m_callouts){
        T->hide(); delete T;
    }
    m_callouts.clear();
}

void View::relationship(float xx){
    std::cout<<__FUNCTION__"("<<xx<<")"<<std::endl;
    QVector<QPointF> truthVec  = m_truthSeries->pointsVector();
    QVector<QPointF> actualVec = m_actualSeries->pointsVector();

    if (truthVec.size() == 0 ) return;
    float leftTruthX = truthVec[0].x(), truthValY;

    int i=1;
    for (; i<truthVec.size(); ++i){
        if (leftTruthX < xx && xx<truthVec[i].x()){
            std::cout<<"X(truth)-wheeled:"<<truthVec[i].x();
            truthValY = truthVec[i].y();
            std::cout<<", Y(truth): "<<truthValY;
            break;
        }
        leftTruthX = truthVec[i].x();
    }
    if (i == truthVec.size()) return;//not found

    if (m_tooltipTruth == NULL )  m_tooltipTruth = new Callout(m_chart);
    QString aux(m_labels.first.c_str());
    aux+=": %1, ";
    aux+=m_labels.second.c_str();
    aux+=": %2";

    m_tooltipTruth->setColor(m_truthSeries->color());
    m_tooltipTruth->setText( aux.arg(leftTruthX, 0, 'f', 6).arg(truthValY, 0, 'f', 6));
    m_tooltipTruth->setAnchor(QPointF(leftTruthX, truthValY));
    m_tooltipTruth->setZValue(11);
    m_tooltipTruth->updateGeometry();
    m_tooltipTruth->show();

    float leftActualX = actualVec[0].x(), actualValY;

    for (i=1; i<actualVec.size(); ++i){
        if (leftActualX < xx && xx<actualVec[i].x()){
            std::cout<<"X(actual):"<<truthVec[i].x();
            actualValY = actualVec[i].y();
            std::cout<<", Y(actual): "<<actualValY;
            break;
        }
        leftActualX = actualVec[i].x();
    }

    if (i == actualVec.size()) return;//not found
    if (m_tooltipActual == NULL )  m_tooltipActual = new Callout(m_chart);
    aux= m_labels.first.c_str();
    aux+=": %1, ";
    aux+=m_labels.second.c_str();
    aux+=": %2";
    m_tooltipActual->setColor(m_actualSeries->color());
    m_tooltipActual->setText( aux.arg(leftActualX, 0, 'f', 6).arg(actualValY, 0, 'f', 6));
    m_tooltipActual->setAnchor(QPointF(leftActualX, actualValY));
    m_tooltipActual->setZValue(11);
    m_tooltipActual->updateGeometry();
    m_tooltipActual->show();
}

void View::connectMarkers(){

    // Connect all markers to handler
    const auto markers = m_chart->legend()->markers();
    for (QLegendMarker *marker : markers) {
        // Disconnect possible existing connection to avoid multiple connections
        QObject::disconnect(marker, &QLegendMarker::clicked,
                            this, &View::handleMarkerClicked);
        QObject::connect(marker, &QLegendMarker::clicked, this, &View::handleMarkerClicked);
    }

}

void View::disconnectMarkers(){

    const auto markers = m_chart->legend()->markers();
    for (QLegendMarker *marker : markers) {
        QObject::disconnect(marker, &QLegendMarker::clicked,
                            this, &View::handleMarkerClicked);
    }

}

void View::handleMarkerClicked(){

    QLegendMarker* marker = qobject_cast<QLegendMarker*> (sender());
    Q_ASSERT(marker);

    switch (marker->type())    {
        case QLegendMarker::LegendMarkerTypeXY:
        {
        // Toggle visibility of series
        marker->series()->setVisible(!marker->series()->isVisible());

        // Turn legend marker back to visible, since hiding series also hides the marker
        // and we don't want it to happen now.
        marker->setVisible(true);
        // Dim the marker, if series is not visible
        qreal alpha = 1.0;

        if (!marker->series()->isVisible())
            alpha = 0.5;

        QColor color;
        QBrush brush = marker->labelBrush();
        color = brush.color();
        color.setAlphaF(alpha);
        brush.setColor(color);
        marker->setLabelBrush(brush);

        brush = marker->brush();
        color = brush.color();
        color.setAlphaF(alpha);
        brush.setColor(color);
        marker->setBrush(brush);

        QPen pen = marker->pen();
        color = pen.color();
        color.setAlphaF(alpha);
        pen.setColor(color);
        marker->setPen(pen);

        break;
        }
    default:
        {
        std::cerr << "Unknown marker type"<<std::endl;
        break;
        }
    }
}

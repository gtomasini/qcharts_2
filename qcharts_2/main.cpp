#include <QApplication>
#include <QtCharts/QSplineSeries>
#include <QCommandLineParser>
#include <QCommandLineOption>

#include <utility>

#include "view.h"
#include "trackutil.h"
//#include "mainwindow.h"
#include "udpServer.h"
#include "dialog.h"


int main(int argc, char *argv[]){
    QApplication app(argc, argv);

    QCoreApplication::setApplicationName("QChart II");
    QCoreApplication::setOrganizationName("Boeing");
    QCoreApplication::setApplicationVersion(QT_VERSION_STR);

    QCommandLineParser parser;
    parser.setApplicationDescription("qchart22 track chart");
    parser.addHelpOption();
    parser.addVersionOption();

    //qDebug() << "App Path : " << qApp->applicationDirPath();
    QCommandLineOption binfile(QStringList() << "f" << "binfile",
              QCoreApplication::translate("main", "process input binfile"),
              QCoreApplication::translate("main", "binfile"), "");
    parser.addOption(binfile);

    //adsb option
    QCommandLineOption adsb(QStringList() << "a" << "adsb",
             QCoreApplication::translate("main", "adsb: all, alt, nsv, esv, gta, lati, lon, latlon"),
             QCoreApplication::translate("main", "adsb"), "");
    parser.addOption(adsb);

    //radar option
    QCommandLineOption radar(QStringList() << "r" << "radar",
             QCoreApplication::translate("main", "radar: all, relran, relalt, relbear"),
             QCoreApplication::translate("main", "radar"), "");
    parser.addOption(radar);

    parser.process(app);

    const QStringList args = parser.optionNames();
    QString ifilename=parser.value(binfile);
    QString adsbStr=parser.value(adsb);
    QString radarStr=parser.value(radar);

    std::cout<<"bin input file name:"<<ifilename.toStdString()<<std::endl;
    std::cout<<"adsb option:"<<adsbStr.toStdString()<<std::endl;
    std::cout<<"radar option:"<<radarStr.toStdString()<<std::endl;

    if (args.size() < 1) {
        std::cerr<<qPrintable(QCoreApplication::translate("main",
                "Warning: should specify arguments. "))<<std::endl;
    }

    /* old qt program ports configuration
    std::make_pair("adsb", "2200"),
    std::make_pair("camera", "2202"),
    std::make_pair("radar", "2201"),
    std::make_pair("clean", "2300"),
    std::make_pair("ownship", "2100")
    */

    udpServer myAdsbServer(adsbPort);
    myAdsbServer.initSocket();

    udpServer myRadarServer(radarPort);
    myRadarServer.initSocket();

    udpServer myTruthServer(truthPort);
    myTruthServer.initSocket();

    trackUtil tu(ifilename.toStdString());

    seriesPair altSeries, nsvSeries, esvSeries, grTrAngleSeries,
            latSeries, lonSeries, latLonSeries;

    Dialog dialog(adsbPort, radarPort, truthPort);

    QObject::connect(&dialog, SIGNAL(signalAdsbPort(int)),
                        &myAdsbServer, SLOT(setPort(int)));
    QObject::connect(&dialog, SIGNAL(signalRadarPort(int)),
                        &myRadarServer, SLOT(setPort(int)));
    QObject::connect(&dialog, SIGNAL(signalTruthPort(int)),
                        &myTruthServer, SLOT(setPort(int)));
    dialog.show();

    //----------------------------"Alt"-------------------------------------------
    if ( adsbStr.left(3).toLower()=="alt" || adsbStr.left(3).toLower()=="all")
        tu.getMagnitudeFromADSBorRADAR<geometricAltitudeAdsb>(altSeries, true);

    View altView(altSeries, std::make_pair("t(s)", "Alt(m)"), View::Plot::alt);
    QObject::connect(&myAdsbServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                        &altView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&myTruthServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                        &altView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&altView, SIGNAL(signalData(double, double , const std::string &)),
                     &dialog, SLOT(showData(double, double, const std::string &)));
    dialog.setAltAdsbView(altView);
    QObject::connect(&altView, SIGNAL(enableAdsb(bool)), &dialog, SLOT(setEnableAdsbButtons(bool)));

    if (adsbStr.left(3).toLower()=="alt" || adsbStr.left(3).toLower()=="all"){
        altView.show();
    }

    //------------------------------"NSV"-----------------------------------------
    if (adsbStr.left(3).toLower()=="nsv" || adsbStr.left(3).toLower()=="all")
        tu.getMagnitudeFromADSBorRADAR<groundSpeedNorthAdsb>(nsvSeries);

    View nsvView(nsvSeries, std::make_pair("t(s)", "NSV(m/s)"), View::Plot::nsv);

    QObject::connect(&myAdsbServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &nsvView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&myTruthServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &nsvView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&nsvView, SIGNAL(signalData(double, double , const std::string &)),
                     &dialog, SLOT(showData(double, double, const std::string &)));
    dialog.setNsvAdsbView(nsvView);

    if (adsbStr.left(3).toLower()=="nsv" || adsbStr.left(3).toLower()=="all"){
        nsvView.show();
    }

    //------------------------------"ESV"----------------------------------------
    if (adsbStr.left(3).toLower()=="esv" || adsbStr.left(3).toLower()=="all")
        tu.getMagnitudeFromADSBorRADAR<groundSpeedEastAdsb>(esvSeries);

    View esvView(esvSeries, std::make_pair("t(s)", "ESV(m/s)"), View::Plot::esv);
    QObject::connect(&myAdsbServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &esvView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&myTruthServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &esvView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&esvView, SIGNAL(signalData(double, double , const std::string &)),
                     &dialog, SLOT(showData(double, double, const std::string &)));

    dialog.setEsvAdsbView(esvView);
    if (adsbStr.left(3).toLower()=="esv" || adsbStr.left(3).toLower()=="all"){
        esvView.show();
    }

    //---------------------------"GroundTrackAngle"------------------------------
    if (adsbStr.left(3).toLower()=="gta" || adsbStr.left(3).toLower()=="all")
        tu.getMagnitudeFromADSBorRADAR<groundTrackAngleAdsb>(grTrAngleSeries);
    View grTrAngleView(grTrAngleSeries, std::make_pair("t(s)", "GroundTrackAngle(deg)"), View::Plot::gta);
    QObject::connect(&myAdsbServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &grTrAngleView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&myTruthServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &grTrAngleView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&grTrAngleView, SIGNAL(signalData(double, double , const std::string &)),
                     &dialog, SLOT(showData(double, double, const std::string &)));
    dialog.setGrTrAngleAdsbView(grTrAngleView);
    if (adsbStr.left(3).toLower()=="gta" || adsbStr.left(3).toLower()=="all"){
        grTrAngleView.show();
    }

    //-------------------------------"latitude"----------------------------------
    if (adsbStr.left(4).toLower()=="lati" || adsbStr.left(3).toLower()=="all")
        tu.getMagnitudeFromADSBorRADAR<latitudeAdsb>(latSeries);
    View latView(latSeries, std::make_pair("t(s)", "Latitude(deg)"), View::Plot::lati );
    QObject::connect(&myAdsbServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &latView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&myTruthServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &latView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&latView, SIGNAL(signalData(double, double , const std::string &)),
                     &dialog, SLOT(showData(double, double, const std::string &)));
    dialog.setLatAdsbView(latView);
    if (adsbStr.left(4).toLower()=="lati" || adsbStr.left(3).toLower()=="all"){
        latView.show();
    }

    //-------------------------------"longitude"---------------------------------
    if (adsbStr.left(3).toLower()=="lon" || adsbStr.left(3).toLower()=="all")
        tu.getMagnitudeFromADSBorRADAR<longitudeAdsb>(lonSeries);
    View lonView(lonSeries, std::make_pair("t(s)", "Longitude(deg)"), View::Plot::lon );
    QObject::connect(&myAdsbServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &lonView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&myTruthServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &lonView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&lonView, SIGNAL(signalData(double, double , const std::string &)),
                     &dialog, SLOT(showData(double, double, const std::string &)));
    dialog.setLonAdsbView(lonView);
    if (adsbStr.left(3).toLower()=="lon" || adsbStr.left(3).toLower()=="all"){
        lonView.show();
    }

    //-------------------------------"LAT-LON"---------------------------------
    if (adsbStr.left(6).toLower()=="latlon" || adsbStr.left(3).toLower()=="all")
        tu.getMagnitudeFromADSBorRADAR<latitudeLongitudeAdsb>(latLonSeries);
    View latLonView(latLonSeries, std::make_pair("LAT(deg)", "LON(deg)"), View::Plot::latlon );
    QObject::connect(&myAdsbServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &latLonView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&myTruthServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &latLonView, SLOT(handleNewTrack(QByteArray&, int)));

    dialog.setLatLonAdsbView(latLonView);
    if (adsbStr.left(6).toLower()=="latlon" || adsbStr.left(3).toLower()=="all"){
        latLonView.show();
    }

    //================================================================================
    //=========================================RADAR==================================
    //
    //
    seriesPair relRanSeries, relAltSeries, relBearSeries;

    //-------------------------------"REL RAN"---------------------------------
    if (radarStr.left(6).toLower()=="relran" || radarStr.left(3).toLower()=="all")
        tu.getMagnitudeFromADSBorRADAR<relativeRangeFromRadar>(relRanSeries, false);
    View relRanView(relRanSeries, std::make_pair("t(s)", "relative range(m)"),
                    View::Plot::relran, false);
    QObject::connect(&myAdsbServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &relRanView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&myRadarServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &relRanView, SLOT(handleNewTrack(QByteArray&, int)));

    QObject::connect(&myTruthServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &relRanView, SLOT(handleNewTrack(QByteArray&, int)));

    QObject::connect(&relRanView, SIGNAL(signalData(double, double , const std::string &)),
                     &dialog, SLOT(showData(double, double, const std::string &)));

    dialog.setRelRanRadarView(relRanView);

    QObject::connect(&relRanView, SIGNAL(enableRadar(bool)), &dialog, SLOT(setEnableRadarButtons(bool)));

    if (radarStr.left(6).toLower()=="relran" || radarStr.left(3).toLower()=="all"){
        relRanView.show();
    }

    //-------------------------------"REL ALT"---------------------------------
    if (radarStr.left(6).toLower()=="relalt" || radarStr.left(3).toLower()=="all")
        tu.getMagnitudeFromADSBorRADAR<relativeAltitudeFromRadar>(relAltSeries, false);
    View relAltView(relAltSeries, std::make_pair("t(s)", "relative altitude(m)"),
                    View::Plot::relalt, false);
    QObject::connect(&myAdsbServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &relAltView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&myRadarServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &relAltView, SLOT(handleNewTrack(QByteArray&, int)));

    QObject::connect(&myTruthServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &relAltView, SLOT(handleNewTrack(QByteArray&, int)));

    QObject::connect(&relAltView, SIGNAL(signalData(double, double , const std::string &)),
                     &dialog, SLOT(showData(double, double, const std::string &)));

    dialog.setRelAltRadarView(relAltView);
    if (radarStr.left(6).toLower()=="relalt" || radarStr.left(3).toLower()=="all"){
        relAltView.show();
    }

    //-------------------------------"REL BEAR"---------------------------------
    if (radarStr.left(6).toLower()=="relbear" || radarStr.left(3).toLower()=="all")
        tu.getMagnitudeFromADSBorRADAR<relativeBearingFromRadar>(relBearSeries, false);
    View relBearView(relBearSeries, std::make_pair("t(s)", "relative bearing(deg)"),
                    View::Plot::relbear, false);
    QObject::connect(&myAdsbServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &relBearView, SLOT(handleNewTrack(QByteArray&, int)));
    QObject::connect(&myRadarServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &relBearView, SLOT(handleNewTrack(QByteArray&, int)));

    QObject::connect(&myTruthServer, SIGNAL(newDatagramSignal(QByteArray&, int)),
                 &relBearView, SLOT(handleNewTrack(QByteArray&, int)));

    QObject::connect(&relBearView, SIGNAL(signalData(double, double , const std::string &)),
                     &dialog, SLOT(showData(double, double, const std::string &)));

    dialog.setRelBearRadarView(relBearView);
    if (radarStr.left(6).toLower()=="relbear" || radarStr.left(3).toLower()=="all"){
        relBearView.show();
    }

    if (!tu.getDataAppended()){
        std::cout<<"use --help for help."<<std::endl;
    }

    //MainWindow mainWin;
    //mainWin.show();
    return app.exec();
}

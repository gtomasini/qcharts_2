#include <QtWidgets>
#include <iostream>
#include "dialog.h"
#include "trackutil.h"
#include "mops2track.h"
#include "daa_datatypes_icd/trackICD.h"

Dialog::Dialog(int _adsbPort, int _radarPort, int _truthPort):adsbPort(_adsbPort),
    radarPort(_radarPort), truthPort(_truthPort){
    createMenu();
    createRadarGroupBox();
    createAdsbGroupBox();

    buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok
                                     | QDialogButtonBox::Cancel);

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
    QVBoxLayout *mainLayout = new QVBoxLayout;

    mainLayout->setMenuBar(menuBar);

    mainLayout->addWidget(radarGroupBox);
    mainLayout->addWidget(adsbGroupBox);
    mainLayout->addWidget(buttonBox);

    infoText = new QTextEdit();
    infoText->verticalScrollBar();
    mainLayout->addWidget(infoText);

    seriesStatus();
    infoText->append(QString("adsb listening in udp port %1...").arg(adsbPort));
    infoText->append(QString("radar listening in udp port %1...").arg(radarPort));
    infoText->append(QString("truth listening in udp port %1...").arg(truthPort));
    infoText->append(tr("load data or send it to udp port in order to enable buttons."));

    statusBar= new QStatusBar();
    mainLayout->addWidget(statusBar);

    statusBar->showMessage(tr("Status Bar..."));

    setLayout(mainLayout);
    setEnableAdsbButtons(false);
    setEnableRadarButtons(false);
    setWindowTitle(tr("Qcharts Control"));
}

void Dialog::createMenu(){
    menuBar = new QMenuBar;
    fileMenu = new QMenu(tr("&File"), this);
    chartMenu = new QMenu(tr("&Chart"), this);
    toolMenu = new QMenu(tr("&Tools"), this);
    helpMenu = new QMenu(tr("&Help"), this);

    menuBar->addMenu(fileMenu);
    menuBar->addMenu(chartMenu);
    menuBar->addMenu(toolMenu);
    menuBar->addSeparator();

    menuBar->addMenu(helpMenu);

    openAdsbCsvAct  = fileMenu->addAction(tr("open &adsb Mops (csv)"));
    openAdsbCsvAct->setStatusTip(tr("open adsb mops file"));
    openRadarCsvAct = fileMenu->addAction(tr("open &radar Mops (csv)"));
    openTruthCsvAct = fileMenu->addAction(tr("open &truth Mops (csv)"));
    fileMenu->addSeparator();
    openBinaryAct   = fileMenu->addAction(tr("&open Binary Track File"));
    openBinTruthAct = fileMenu->addAction(tr("open Binary Track File As Truth"));
    fileMenu->addSeparator();
    saveBinFileAct  = fileMenu->addAction(tr("&save Binary Track File"));
    fileMenu->addSeparator();
    exitAct         = fileMenu->addAction(tr("e&Xit"));

    cleanActualAct  = chartMenu->addAction(tr("clean &Actual Series"));
    cleanTruthAct   = chartMenu->addAction(tr("clean &Truth Series"));
    cleanLabelsAct  = chartMenu->addAction(tr("clean &Label Series"));
    chartMenu->addSeparator();
    cleanVecAct     = chartMenu->addAction(tr("clean Internal Container"));

    setAdsbPortAct  = toolMenu->addAction(tr("set &Adsb Port"));
    setRadarPortAct = toolMenu->addAction(tr("set &Radar Port"));
    setTruthPortAct = toolMenu->addAction(tr("set &Truth Port"));
    seriesStatusAct = toolMenu->addAction(tr("get series &Status"));

    fileHelpAct     = helpMenu->addAction(tr("&File"));
    chartHelpAct    = helpMenu->addAction(tr("&Chart"));
    generalHelpAct  = helpMenu->addAction(tr("&General"));
    chartMenu->addSeparator();
    aboutAct        = helpMenu->addAction(tr("&About"));
    aboutAct->setStatusTip(tr("Show the application's About box"));

    connect(openAdsbCsvAct, SIGNAL(triggered()), this, SLOT(openAdsbCsv()));
    connect(openRadarCsvAct, SIGNAL(triggered()), this, SLOT(openRadarCsv()));
    connect(openTruthCsvAct, SIGNAL(triggered()), this, SLOT(openTruthCsv()));
    connect(openBinaryAct, SIGNAL(triggered()), this, SLOT(openBinary()));
    connect(openBinTruthAct, SIGNAL(triggered()), this, SLOT(openBinaryAsTruth()));
    connect(saveBinFileAct, SIGNAL(triggered()), this, SLOT(saveBinaryFile()));
    connect(exitAct, SIGNAL(triggered()), this, SLOT(reject()));

    connect(cleanActualAct, SIGNAL(triggered()), this, SLOT(cleanActualSeries()));
    connect(cleanTruthAct, SIGNAL(triggered()), this, SLOT(cleanTruthSeries()));
    connect(cleanLabelsAct, SIGNAL(triggered()), this, SLOT(cleanLabels()));
    connect(cleanVecAct, SIGNAL(triggered()), this, SLOT(cleanVector()));

    connect(setAdsbPortAct, SIGNAL(triggered()), this, SLOT(setAdsbPort()));
    connect(setRadarPortAct, SIGNAL(triggered()), this, SLOT(setRadarPort()));
    connect(setTruthPortAct, SIGNAL(triggered()), this, SLOT(setAdsbPort()));
    connect(seriesStatusAct, SIGNAL(triggered()), this, SLOT(seriesStatus()));

    connect(fileHelpAct, SIGNAL(triggered()), this, SLOT(fileHelpText()));
    connect(chartHelpAct, SIGNAL(triggered()), this, SLOT(chartHelpText()));
    connect(generalHelpAct, SIGNAL(triggered()), this, SLOT(generalHelpText()));

    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

}

void Dialog::setAdsbPort(){
    bool ok;
    adsbPort = QInputDialog::getInt(nullptr, "Adsb Port",
                "udp port:", adsbPort, 2000, 3000, 1, &ok);
    if (ok){
        std::cout << "User entered Adsb port: " << adsbPort << std::endl;
        emit signalAdsbPort(adsbPort);
    }
}

void Dialog::setRadarPort(){
    bool ok;
    radarPort = QInputDialog::getInt(nullptr, "Radar Port",
                "udp port:", radarPort, 2000, 3000, 1, &ok);
    if (ok){
        std::cout << "User entered Radar port: " << radarPort << std::endl;
        emit signalRadarPort(radarPort);
    }
}

void Dialog::setTruthPort(){
    bool ok;
    adsbPort = QInputDialog::getInt(nullptr, "Adsb Port",
                "udp port:", adsbPort, 2000, 3000, 1, &ok);
    if (ok){
        std::cout << "User entered Truth port: " << adsbPort << std::endl;
        emit signalAdsbPort(adsbPort);
    }
}

void Dialog::chartHelpText(){
    QMessageBox::information(this, tr("Chart Help"),
        tr( "<b>clean Internal Container:</b> clean container used for store csv filess data.<br>"
            "<b>clean Actual Series:</b> clean qchart actual serie.<br>"
            "<b>clean Truth Series:</b> clean qchart truth serie.<br>"
            "<b>clean Label Series:</b> clean qchart labels.<br>"
           ));
}

void Dialog::generalHelpText(){
    QMessageBox::information(this, tr("General Help"),
        tr("<b><u>Mouse:</u></b><br>"
        "<b><i>arrow hover the curves:</i></b> display x-y pair.<br>"
        "<b><i>left button pressed:</i></b> over the curves labels x-y pair permanentely.<br>"
        "<b><i>wheel pressed:</i></b> matches y-values of both curves at that x coordinate.<br>"
        "<b><i>wheel spin:</i></b> zoom in/out.<br>"
        "<b><i>left button pressed and move the mouse:</i></b> scroll in the same direction.<br>"
        "================================================================<br>"
        "<b><u>Keys:</u></b><br>"
        "<b><i>+/-:</i></b> zoom in/out.<br>"
        "<b><i>arrows:</i></b> scroll.<br>"
        "<b><i>F6:</i></b> clean series Labels (see Mouse).<br>"
        "<b><i>F7:</i></b> clean actual curve(serie) data.<br>"
        "<b><i>F8:</i></b> clean truth curve(serie) data.<br>"
        "================================================================<br>"
        "<b><u>Notes:</u></b><br>"
        "An internal container (vector) is used in order to store csv data(mops)."
        "Every time a new csv file is read, the data is appended to it "
        "and it is reorderered (using time field). It can be cleared from Charts"
        " Menu. Output Binary file will be created with this vector data.<br>"
        "Truth and Actual curves are displayed using QLineSeries (two series are used respectively). "
        "So the data is copied from vector container to series before plotting.<br>"
        "There is an udp server listening in port 2200(adsb, radar, truth), etc. in order to receive binary track data"
    ));
}

void Dialog::about(){
    QMessageBox::about(this, tr("QCharts II"),
            tr("QCharts II. V2.0, Boeing"));
}


void Dialog::fileHelpText(){
    QMessageBox::information(this, tr("File Help"),
      tr("<b>Open Adsb Mops (csv):</b> opens csv adsb and parse it, puts data in container and actual series.<br>"
         "<b>Open Radar Mops (csv):</b> opens csv radar and parse it, puts data in container and actual series.<br>"
         "<b>Open Truth Mops (csv):</b> opens csv adsb and parse it, puts data in container and truth series.<br>"
         "<b>Open Binary Track File:</b> opens binary track file and puts in container and series.<br>"
         "<b>Open Binary Track File As Truth:</b> opens binary track file and puts in container and actual series.<br>"
         "<b>Save Binary Track File:</b> saves container to Binary Track File.<br>"
     ));
}

void Dialog::saveBinaryFile(){
    if (trackVec.size()==0){
         QMessageBox::information(this, tr("Container size = 0"), tr("please load some data from csv file"));
         return;
    }
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Binary File"),
        "untitled.bin",
        tr("bin (*.bin);;All Files (*)"));
    if (fileName.isEmpty())  return;

    int st=mops2track::save2Bin(trackVec, fileName.toStdString());
    if (st == -1){
        QMessageBox::information(this, tr("Unable to open file "), fileName);
        return;
    }

    statusBar->showMessage(QString("%1 tracks saved to %2").arg((int)trackVec.size()).arg(fileName));
    infoText->append(QString("%1 tracks saved to %2").arg((int)trackVec.size()).arg(fileName));
}

void Dialog::openAdsbCsv(){
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Mops Adsb input file"), "",
        tr("bin (*.csv);;All Files (*)"));
    if (fileName.isEmpty())  return;

    //std::vector <trackICD::track> trackVec;//big vector where store all data from all sources
    int st=mops2track::readCsvFileAndSave(fileName.toStdString(), trackVec, mops2track::source::adsb);

    if (st == -1){
        QMessageBox::information(this, tr("Unable to open file "), fileName);
        return;
    }
    mops2track::orchestrator(trackVec);

    std::cout<<"adsb track Vector read size:"<<trackVec.size()<<std::endl;

    statusBar->showMessage(QString("adsb track Vector read size: %1").arg((int)trackVec.size()));
    infoText->append("adsb mops data loaded into internal container.");

    trackUtil tu(trackVec);

    setEnableAdsbButtons(true);

    getAdsbData(tu);
}

void Dialog::openRadarCsv(){
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Mops Radar input file"), "",
        tr("bin (*.csv);;All Files (*)"));
    if (fileName.isEmpty())  return;

    //std::vector <trackICD::track> trackVec;//big vector where store all data from all sources
    int st=mops2track::readCsvFileAndSave(fileName.toStdString(), trackVec, mops2track::source::radar);
    if (st == -1){
        QMessageBox::information(this, tr("Unable to open file "), fileName);
        return;
    }
    mops2track::orchestrator(trackVec);

    std::cout<<"radar track Vector read size:"<<trackVec.size()<<std::endl;

    statusBar->showMessage(QString("radar track Vector read size: %1").arg((int)trackVec.size()));
    infoText->append("radar mops data loaded into internal container.");

    trackUtil tu(trackVec);

    setEnableRadarButtons(true);

    getRadarData(tu);
}

void Dialog::openTruthCsv(){
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Mops Truth input file"), "",
        tr("bin (*.csv);;All Files (*)"));
    if (fileName.isEmpty())  return;

    //std::vector <trackICD::track> trackVec;//big vector where store all data from all sources
    int st= mops2track::readCsvFileAndSave(fileName.toStdString(), trackVec, mops2track::source::truth);
    if (st == -1){
        QMessageBox::information(this, tr("Unable to open file "), fileName);
        return;
    }
    mops2track::orchestrator(trackVec);

    std::cout<<"truth track Vector read size:"<<trackVec.size()<<std::endl;

    statusBar->showMessage(QString("truth track Vector read size: %1").arg((int)trackVec.size()));
    infoText->append("truth mops data loaded into internal container.");
    trackUtil tu(trackVec);

    setEnableAdsbButtons(true);
    setEnableRadarButtons(true);

    getAdsbData(tu);
    getRadarData(tu);
}


void Dialog::getAdsbData(trackUtil &tu){
    if (altAdsbView != nullptr){
        tu.getMagnitudeFromADSBorRADAR<geometricAltitudeAdsb>(
            altAdsbView->getTruthSeries(), altAdsbView->getActualSeries(), true);
    }

    if (nsvAdsbView != nullptr){
        tu.getMagnitudeFromADSBorRADAR<groundSpeedNorthAdsb>(
            nsvAdsbView->getTruthSeries(), nsvAdsbView->getActualSeries(), true);
    }

    if (esvAdsbView != nullptr){
        tu.getMagnitudeFromADSBorRADAR<groundSpeedEastAdsb>(
            esvAdsbView->getTruthSeries(), esvAdsbView->getActualSeries(), true);
    }

    if (grTrAngleAdsbView != nullptr){
        tu.getMagnitudeFromADSBorRADAR<groundTrackAngleAdsb>(
             grTrAngleAdsbView->getTruthSeries(), grTrAngleAdsbView->getActualSeries(), true);
    }

    if (latAdsbView !=nullptr){
        tu.getMagnitudeFromADSBorRADAR<latitudeAdsb>(
             latAdsbView->getTruthSeries(), latAdsbView->getActualSeries(), true);
    }

    if (lonAdsbView != nullptr){
        tu.getMagnitudeFromADSBorRADAR<longitudeAdsb>(
             lonAdsbView->getTruthSeries(), lonAdsbView->getActualSeries(), true);
    }

    if (latlonAdsbView != nullptr){
        tu.getMagnitudeFromADSBorRADAR<latitudeLongitudeAdsb>(
             latlonAdsbView->getTruthSeries(), latlonAdsbView->getActualSeries(), true);
    }
}

void Dialog::getRadarData(trackUtil &tu){
    if (relRanRadarView != nullptr){
        tu.getMagnitudeFromADSBorRADAR<relativeRangeFromRadar>(
             relRanRadarView->getTruthSeries(), relRanRadarView->getActualSeries(), false);
    }

    if (relAltRadarView != nullptr){
        tu.getMagnitudeFromADSBorRADAR<relativeAltitudeFromRadar>(
             relAltRadarView->getTruthSeries(), relAltRadarView->getActualSeries(), false);
    }

    if (relBearRadarView != nullptr){
        tu.getMagnitudeFromADSBorRADAR<relativeBearingFromRadar>(
         relBearRadarView->getTruthSeries(), relBearRadarView->getActualSeries(), false);
    }
}

void Dialog::openBinary(){
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Track Binary Input File"), "",
        tr("bin (*.bin);;All Files (*)"));
    if (fileName.isEmpty())  return;

    trackUtil tu(fileName.toStdString());

    if (tu.getFerror()){
        QString err(tu.getLastError().c_str());
        QMessageBox::information(this, tr("Unable to open file"), err);
        return;
    }

    statusBar->showMessage(QString("track Vector read size: %1").arg(tu.getVectorSize()));
    infoText->append("track bin data loaded");

    getAdsbData(tu);
    getRadarData(tu);
    setEnableAdsbButtons(true);
    setEnableRadarButtons(true);
}


void Dialog::openBinaryAsTruth(){
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Binary input file as Truth"), "",
        tr("bin (*.bin);;All Files (*)"));
    if (fileName.isEmpty())  return;

    trackUtil tu(fileName.toStdString());

    if (tu.getFerror()){
        QString err(tu.getLastError().c_str());
        QMessageBox::information(this, tr("Unable to open file"), err);
        return;
    }

    tu.setVectorSourceAsUnknown();//set as truth

    statusBar->showMessage(QString("track Vector read size: %1").arg(tu.getVectorSize()));
    infoText->append("track bin data loaded as truth");

    getAdsbData(tu);
    getRadarData(tu);
    setEnableAdsbButtons(true);
    setEnableRadarButtons(true);
}

void Dialog::cleanActualSeries(){
    std::cout<<"clean actual series"<<std::endl;

    if (altAdsbView != nullptr) altAdsbView->getActualSeries()->clear();

    if (nsvAdsbView != nullptr) nsvAdsbView->getActualSeries()->clear();

    if (esvAdsbView != nullptr) esvAdsbView->getActualSeries()->clear();

    if (grTrAngleAdsbView != nullptr) grTrAngleAdsbView->getActualSeries()->clear();

    if (latAdsbView != nullptr)  latAdsbView->getActualSeries()->clear();

    if (lonAdsbView != nullptr) lonAdsbView->getActualSeries()->clear();

    if (latlonAdsbView !=nullptr) latlonAdsbView->getActualSeries()->clear();

    if (relRanRadarView != nullptr)  relRanRadarView->getActualSeries()->clear();

    if (relAltRadarView != nullptr)  relAltRadarView->getActualSeries()->clear();

    if (relBearRadarView != nullptr)  relBearRadarView->getActualSeries()->clear();

    statusBar->showMessage(tr("actual series cleaned"));
    infoText->append(tr("actual series cleaned"));
}

void Dialog::cleanLabels(){
    std::cout<<"clean label series"<<std::endl;

    if (altAdsbView != nullptr) altAdsbView->cleanLabels();

    if (nsvAdsbView != nullptr) nsvAdsbView->cleanLabels();

    if (esvAdsbView != nullptr) esvAdsbView->cleanLabels();

    if (grTrAngleAdsbView != nullptr) grTrAngleAdsbView->cleanLabels();

    if (latAdsbView != nullptr)  latAdsbView->cleanLabels();

    if (lonAdsbView != nullptr) lonAdsbView->cleanLabels();

    if (latlonAdsbView !=nullptr) latlonAdsbView->cleanLabels();

    if (relRanRadarView != nullptr)  relRanRadarView->cleanLabels();

    if (relAltRadarView != nullptr)  relAltRadarView->cleanLabels();

    if (relBearRadarView != nullptr)  relBearRadarView->cleanLabels();
}


void Dialog::cleanTruthSeries(){
    std::cout<<"clean truth series"<<std::endl;

    if (altAdsbView != nullptr)  altAdsbView->getTruthSeries()->clear();

    if (nsvAdsbView != nullptr) nsvAdsbView->getTruthSeries()->clear();

    if (esvAdsbView != nullptr) esvAdsbView->getTruthSeries()->clear();

    if (grTrAngleAdsbView != nullptr) grTrAngleAdsbView->getTruthSeries()->clear();

    if (latAdsbView !=nullptr) latAdsbView->getTruthSeries()->clear();

    if (lonAdsbView != nullptr) lonAdsbView->getTruthSeries()->clear();

    if (latlonAdsbView !=nullptr) latlonAdsbView->getTruthSeries()->clear();

    if (relRanRadarView != nullptr) relRanRadarView->getTruthSeries()->clear();

    if (relAltRadarView != nullptr) relAltRadarView->getTruthSeries()->clear();

    if (relBearRadarView != nullptr) relBearRadarView->getTruthSeries()->clear();

    statusBar->showMessage(tr("truth series cleaned"));
    infoText->append(tr("truth series cleaned"));
}

void Dialog::seriesStatus(){
    infoText->append(tr("series status"));
    if (altAdsbView != nullptr){
         infoText->append(QString("truth adsb altitude: %1 entries").arg(
                altAdsbView->getTruthSeries()->count()));
         infoText->append(QString("actual adsb altitude: %1 entries").arg(
                altAdsbView->getActualSeries()->count()));
    }

    if (nsvAdsbView != nullptr){
        infoText->append(QString("truth adsb nsv: %1 entries").arg(
               nsvAdsbView->getTruthSeries()->count()));
        infoText->append(QString("actual adsb nsv: %1 entries").arg(
               nsvAdsbView->getActualSeries()->count()));
    }

    if (esvAdsbView != nullptr){
        infoText->append(QString("truth adsb esv: %1 entries").arg(
               esvAdsbView->getTruthSeries()->count()));
        infoText->append(QString("actual adsb esv: %1 entries").arg(
               esvAdsbView->getActualSeries()->count()));
    }

    if (grTrAngleAdsbView != nullptr){
        infoText->append(QString("truth adsb ground track angle: %1 entries").arg(
               grTrAngleAdsbView->getTruthSeries()->count()));
        infoText->append(QString("actual adsb ground track angle: %1 entries").arg(
               grTrAngleAdsbView->getActualSeries()->count()));
    }

    if (latAdsbView !=nullptr){
        infoText->append(QString("truth adsb latitude: %1 entries").arg(
               latAdsbView->getTruthSeries()->count()));
        infoText->append(QString("actual adsb latitude: %1 entries").arg(
               latAdsbView->getActualSeries()->count()));
    }

    if (lonAdsbView != nullptr){
        infoText->append(QString("truth adsb longitude: %1 entries").arg(
               lonAdsbView->getTruthSeries()->count()));
        infoText->append(QString("actual adsb longitude: %1 entries").arg(
               lonAdsbView->getActualSeries()->count()));
    }

    if (latlonAdsbView !=nullptr){
        infoText->append(QString("truth adsb latitude-longitude: %1 entries").arg(
               latlonAdsbView->getTruthSeries()->count()));
        infoText->append(QString("actual adsb latitude-longitude: %1 entries").arg(
               latlonAdsbView->getActualSeries()->count()));
    }

    if (relRanRadarView != nullptr){
        infoText->append(QString("truth radar relative range: %1 entries").arg(
               relRanRadarView->getTruthSeries()->count()));
        infoText->append(QString("actual radar relative range: %1 entries").arg(
               relRanRadarView->getActualSeries()->count()));
    }

    if (relAltRadarView != nullptr){
        infoText->append(QString("truth radar relative altitude: %1 entries").arg(
               relRanRadarView->getTruthSeries()->count()));
        infoText->append(QString("actual radar relative altitude: %1 entries").arg(
               relRanRadarView->getActualSeries()->count()));
    }

    if (relBearRadarView != nullptr){
        infoText->append(QString("truth radar bearing range: %1 entries").arg(
               relBearRadarView->getTruthSeries()->count()));
        infoText->append(QString("actual radar bearing range: %1 entries").arg(
               relBearRadarView->getActualSeries()->count()));
    }

}

void Dialog::showData(double x, double y, const std::string &src){
    infoText->append(QString("%1: udp data received: %2, %3").arg(src.c_str()).arg(x).arg(y));
}


void Dialog::createAdsbGroupBox(){
    adsbGroupBox = new QGroupBox(tr("ADSB"));
    QHBoxLayout *layout = new QHBoxLayout;

    altButton = new QPushButton(tr("Altitude"));
    nsvButton = new QPushButton(tr("NSV"));
    esvButton = new QPushButton(tr("ESV"));
    grTrAngleButton = new QPushButton(tr("GrTrAngle"));
    latButton = new QPushButton(tr("latitude"));
    lonButton = new QPushButton(tr("longitude"));
    latlonButton = new QPushButton(tr("latitude-longitude"));

    layout->addWidget(altButton);
    layout->addWidget(nsvButton);
    layout->addWidget(esvButton);
    layout->addWidget(grTrAngleButton);
    layout->addWidget(latButton);
    layout->addWidget(lonButton);
    layout->addWidget(latlonButton);

    connect(altButton, SIGNAL(pressed()), this, SLOT(altPressed()));
    connect(nsvButton, SIGNAL(pressed()), this, SLOT(nsvPressed()));
    connect(esvButton, SIGNAL(pressed()), this, SLOT(esvPressed()));
    connect(grTrAngleButton, SIGNAL(pressed()), this, SLOT(grTrAnglePressed()));
    connect(latButton, SIGNAL(pressed()), this, SLOT(latPressed()));
    connect(lonButton, SIGNAL(pressed()), this, SLOT(lonPressed()));
    connect(latlonButton, SIGNAL(pressed()), this, SLOT(latlonPressed()));

    adsbGroupBox->setLayout(layout);
}

void Dialog::createRadarGroupBox(){
    radarGroupBox = new QGroupBox(tr("RADAR"));
    QHBoxLayout *layout = new QHBoxLayout;

    relRanButton = new QPushButton(tr("Relative Range"));
    relAltButton = new QPushButton(tr("Relative Altitude"));
    relBearButton = new QPushButton(tr("Relative Bear"));
    //add checkbos
    layout->addWidget(relRanButton);
    layout->addWidget(relAltButton);
    layout->addWidget(relBearButton);

    connect(relRanButton, SIGNAL(pressed()), this, SLOT(relRanPressed()));
    connect(relAltButton, SIGNAL(pressed()), this, SLOT(relAltPressed()));
    connect(relBearButton, SIGNAL(pressed()), this, SLOT(relBearPressed()));
    radarGroupBox->setLayout(layout);
}

void Dialog::setEnableAdsbButtons(bool enabled){
    altButton->setEnabled(enabled);
    nsvButton->setEnabled(enabled);
    esvButton->setEnabled(enabled);
    grTrAngleButton->setEnabled(enabled);
    latButton->setEnabled(enabled);
    lonButton->setEnabled(enabled);
    latlonButton->setEnabled(enabled);
}

void Dialog::setEnableRadarButtons(bool enabled){
    std::cout<<"setEnabledRadarButtons:"<<enabled<<std::endl;
    relRanButton->setEnabled(enabled);
    relAltButton->setEnabled(enabled);
    relBearButton->setEnabled(enabled);
}

void Dialog::altPressed(){
    if (altON){
        altON=false;
        std::cout<<"ALT OFF"<<std::endl;
        if (altAdsbView != nullptr && altAdsbView->isVisible())
            altAdsbView->hide();
    }
    else{
        altON=true;
        std::cout<<"ALT ON"<<std::endl;
        if (altAdsbView != nullptr && !altAdsbView->isVisible())
            altAdsbView->show();
    }
}

void Dialog::nsvPressed(){
    if (nsvON){
        nsvON=false;
        std::cout<<"NSV OFF"<<std::endl;
        if (nsvAdsbView != nullptr && nsvAdsbView->isVisible())
            nsvAdsbView->hide();
    }
    else{
        nsvON=true;
        std::cout<<"NSV ON"<<std::endl;
        if (nsvAdsbView != nullptr && !nsvAdsbView->isVisible())
            nsvAdsbView->show();
    }
}

void Dialog::esvPressed(){
    if (esvON){
        esvON=false;
        std::cout<<"ESV OFF"<<std::endl;
        if (esvAdsbView != nullptr && esvAdsbView->isVisible())
            esvAdsbView->hide();
    }
    else{
        esvON=true;
        std::cout<<"ESV ON"<<std::endl;
        if (esvAdsbView != nullptr && !esvAdsbView->isVisible())
            esvAdsbView->show();
    }
}

void Dialog::grTrAnglePressed(){
    if (grTrAngleON){
        grTrAngleON=false;
        std::cout<<"grTrAngle OFF"<<std::endl;
        if (grTrAngleAdsbView != nullptr && grTrAngleAdsbView->isVisible())
            grTrAngleAdsbView->hide();
    }
    else{
        grTrAngleON=true;
        std::cout<<"grTrAngle ON"<<std::endl;
        if (grTrAngleAdsbView != nullptr && !grTrAngleAdsbView->isVisible())
            grTrAngleAdsbView->show();
    }
}


void Dialog::latPressed(){
    if (latON){
        latON=false;
        std::cout<<"latitude OFF"<<std::endl;
        if (latAdsbView != nullptr && latAdsbView->isVisible())
            latAdsbView->hide();
    }
    else{
        latON=true;
        std::cout<<"altitude ON"<<std::endl;
        if (latAdsbView != nullptr && !latAdsbView->isVisible())
            latAdsbView->show();
    }
}

void Dialog::lonPressed(){
    if (lonON){
        lonON=false;
        std::cout<<"longitude OFF"<<std::endl;
        if (lonAdsbView != nullptr && lonAdsbView->isVisible())
            lonAdsbView->hide();
    }
    else{
        lonON=true;
        std::cout<<"longitude ON"<<std::endl;
        if (lonAdsbView != nullptr && !lonAdsbView->isVisible())
            lonAdsbView->show();
    }
}

void Dialog::latlonPressed(){
    if (latlonON){
        latlonON=false;
        std::cout<<"latitude longitude OFF"<<std::endl;
        if (latlonAdsbView != nullptr && latlonAdsbView->isVisible())
            latlonAdsbView->hide();
    }
    else{
        latlonON=true;
        std::cout<<"latitude longitude ON"<<std::endl;
        if (latlonAdsbView != nullptr && !latlonAdsbView->isVisible())
            latlonAdsbView->show();
    }
}

void Dialog::relRanPressed(){
    if (relRanON){
        relRanON=false;
        std::cout<<"Rel Range OFF"<<std::endl;
        if (relRanRadarView != nullptr && relRanRadarView->isVisible())
            relRanRadarView->hide();
    }
    else{
        relRanON=true;
        std::cout<<"Rel Range ON"<<std::endl;
        if (relRanRadarView != nullptr && !relRanRadarView->isVisible())
            relRanRadarView->show();
    }
}

void Dialog::relAltPressed(){
    if (relAltON){
        relAltON=false;
        std::cout<<"Rel Altitude OFF"<<std::endl;
        if (relAltRadarView != nullptr && relAltRadarView->isVisible())
            relAltRadarView->hide();
    }
    else{
        relAltON=true;
        std::cout<<"Rel Altitude ON"<<std::endl;
        if (relAltRadarView != nullptr && !relAltRadarView->isVisible())
            relAltRadarView->show();
    }
}

void Dialog::relBearPressed(){
    if (relBearON){
        relBearON=false;
        std::cout<<"Rel Bear OFF"<<std::endl;
        if (relBearRadarView != nullptr && relBearRadarView->isVisible())
            relBearRadarView->hide();
    }
    else{
        relBearON=true;
        std::cout<<"Rel Bear ON"<<std::endl;
        if (relBearRadarView != nullptr && !relBearRadarView->isVisible())
            relBearRadarView->show();
    }
}

void Dialog::accept(){
    setEnableAdsbButtons(true);
    setEnableRadarButtons(true);
    about();
}

void Dialog::reject(){
    std::cout<<"CANCEL!!!!"<<std::endl;
    if (altAdsbView != nullptr) altAdsbView->close();
    if (nsvAdsbView != nullptr) nsvAdsbView->close();
    if (esvAdsbView != nullptr) esvAdsbView->close();
    if (grTrAngleAdsbView != nullptr) grTrAngleAdsbView->close();
    if (latAdsbView != nullptr) latAdsbView->close();
    if (lonAdsbView != nullptr) lonAdsbView->close();
    if (latlonAdsbView != nullptr) latlonAdsbView->close();

    if (relRanRadarView != nullptr) relRanRadarView->close();
    if (relAltRadarView != nullptr) relAltRadarView->close();
    if (relBearRadarView != nullptr) relBearRadarView->close();

    QDialog::reject();
}




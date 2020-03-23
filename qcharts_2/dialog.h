#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QAction>
#include <vector>
#include "view.h"
#include "trackutil.h"

QT_BEGIN_NAMESPACE
class QAction;
class QDialogButtonBox;
class QGroupBox;
class QLabel;
class QMenu;
class QMenuBar;
class QPushButton;
class QCheckBox;
class QStatusBar;
class QTextEdit;
QT_END_NAMESPACE

class DialogOptionsWidget;

class Dialog : public QDialog{
    Q_OBJECT

public:
    Dialog(int _adsbPort=2200, int _radarPort=2200, int _truthPort=2200);

    void setAltAdsbView(View &v){altAdsbView=&v;}
    void setNsvAdsbView(View &v){nsvAdsbView=&v;}
    void setEsvAdsbView(View &v){esvAdsbView=&v;}
    void setGrTrAngleAdsbView(View &v){grTrAngleAdsbView=&v;}
    void setLatAdsbView(View &v){latAdsbView=&v;}
    void setLonAdsbView(View &v){lonAdsbView=&v;}
    void setLatLonAdsbView(View &v){latlonAdsbView=&v;}
    void setRelAltRadarView(View &v){relAltRadarView=&v;}
    void setRelRanRadarView(View &v){relRanRadarView=&v;}
    void setRelBearRadarView(View &v){relBearRadarView=&v;}

private slots:
    void accept();
    void reject();
    void openBinary();
    void openAdsbCsv();
    void openRadarCsv();
    void openTruthCsv();
    void saveBinaryFile();
    void openBinaryAsTruth();
    void cleanActualSeries();
    void cleanTruthSeries();
    void cleanLabels();
    void altPressed();
    void nsvPressed();
    void esvPressed();
    void grTrAnglePressed();
    void latPressed();
    void lonPressed();
    void latlonPressed();
    void relRanPressed();
    void relAltPressed();
    void relBearPressed();
    void setEnableAdsbButtons(bool enabled=true);
    void setEnableRadarButtons(bool enabled=true);
    void cleanVector(){ trackVec.clear();}
    void setAdsbPort();
    void setRadarPort();
    void setTruthPort();
    void fileHelpText();
    void chartHelpText();
    void generalHelpText();
    void about();
    void seriesStatus();
    void showData(double x, double y, const std::string  &src);

signals:
    void signalAdsbPort(int port);
    void signalRadarPort(int port);
    void signalTruthPort(int port);

private:
    void createMenu();
    void createAdsbGroupBox();
    void createRadarGroupBox();
    void createToolBox();
    void getAdsbData(trackUtil &tu);
    void getRadarData(trackUtil &tu);

    QMenuBar *menuBar;
    QGroupBox *radarGroupBox, *adsbGroupBox;
    //ADSB buttons
    QPushButton *altButton, *nsvButton, *esvButton, *grTrAngleButton,
            *latButton, *lonButton, *latlonButton;
    //RADAR Buttons
    QPushButton *relRanButton, *relAltButton, *relBearButton;
    //OK Buttons
    QDialogButtonBox *buttonBox;
    QTextEdit *infoText;
    QStatusBar *statusBar;

    QMenu *fileMenu, *chartMenu, *toolMenu, *helpMenu;

    QAction *openAdsbCsvAct, *openRadarCsvAct, *openTruthCsvAct;
    QAction *exitAct, *openBinaryAct, *openBinTruthAct, *saveBinFileAct;
    QAction *cleanActualAct, *cleanTruthAct, *cleanLabelsAct, *cleanVecAct;
    QAction *setAdsbPortAct, *setRadarPortAct, *setTruthPortAct, *seriesStatusAct;
    QAction *fileHelpAct, *chartHelpAct, *generalHelpAct, *aboutAct;
    bool accepted=false;

    View *altAdsbView=nullptr, *nsvAdsbView=nullptr, *esvAdsbView=nullptr,
        *grTrAngleAdsbView=nullptr, *latAdsbView=nullptr, *lonAdsbView=nullptr,
        *latlonAdsbView=nullptr;
    View *relRanRadarView=nullptr, *relAltRadarView=nullptr, *relBearRadarView=nullptr;

    bool altON=false, nsvON=false, esvON=false, grTrAngleON=false,
        latON=false, lonON=false, latlonON=false;
    bool relRanON=false, relAltON=false, relBearON=false;
    std::vector <trackICD::track> trackVec;
    int adsbPort=2200, radarPort=2200, truthPort=2200;
};

#endif

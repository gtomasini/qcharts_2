#include <sstream>
#include <iostream>
#include <fstream>
#include <QString>
#include <QRegExp>
#include <QStringList>
#include "parseCsv.h"

static QRegExp rx("(\\ |\\,|\\;\\t)"); //RegEx for ' ' or ',' or ':' or '\t'

parseCsv::parseCsv(const std::string &ifname) {
	iStream.open(ifname);
}

//only reads header 
std::vector<std::string> parseCsv::readHeader() {
	std::vector<std::string> fields;
	if (iStream.is_open() == false) return fields;
	iStream.seekg(0, std::ios::beg);

    std::string header;
	iStream >> header;
    QString qsHeader(header.c_str());
    //StringTokenizer tokenizer(header, separator);

    QStringList query = qsHeader.split(rx);

    for (int i = 0; i < query.count(); ++i)
        fields.push_back(query.at(i).toStdString());

	headerV = fields;
	return fields;
}

bool parseCsv::readRow(std::map<std::string, std::string> &row) {
	std::string irow;
	iStream >> irow;
    if (iStream.eof()) return false;
    QString qsIrow(irow.c_str());
	
    //StringTokenizer tokenizer(irow, separator);
    QStringList query = qsIrow.split(rx);

    for (int i=0; i < query.count(); ++i)
        row[headerV[i]] = query.at(i).toStdString();

	return true;
}


parseCsv::~parseCsv(){
	iStream.close();
}

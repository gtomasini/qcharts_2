#pragma once
#include <string>
#include <vector>
#include <map>
#include <fstream>

class parseCsv{
	std::ifstream	iStream;
	std::vector<std::string>  headerV;
    //const std::string separator=",";
	
public:
	parseCsv(const std::string &ifname);//opens file fname

	std::vector<std::string> readHeader();//readHeader and fills field2Col

	bool fileIsOpen() const {
		return iStream.is_open();
	}

	//return false if EOF reached
	bool readRow(std::map<std::string, std::string> &row);
	
	bool endOfFile() const{
		return iStream.eof();
	}

	std::vector<std::string>  getHeaderV() const {
		return headerV;
	}

	virtual ~parseCsv();//close file
};


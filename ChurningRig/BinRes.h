#ifndef _waxie_binary_resource_h_
#define _waxie_binary_resource_h_

class BinRes  
{
public:
	BinRes();
	virtual ~BinRes();

public:
	static void ExtractBinResource( std::wstring strCustomResName, int nResourceId, std::wstring strOutputName);
	static void BinRes::ExtractBinResource(std::wstring strCustomResName, int nResourceId, std::wstring strOutputName, std::wstring path);

private:
	static std::wstring getAppLocation();

};

#endif 

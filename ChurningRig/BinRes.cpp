#include "pch.h"
#include "Windows.h"
#include "BinRes.h"


#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif


BinRes::BinRes()
{

}

BinRes::~BinRes()
{

}

void BinRes::ExtractBinResource( std::wstring strCustomResName, int nResourceId, std::wstring strOutputName )
{
	HGLOBAL hResourceLoaded;		// handle to loaded resource 
	HRSRC hRes;						// handle/ptr. to res. info. 
	char *lpResLock;				// pointer to resource data 
	DWORD dwSizeRes;
	std::wstring strOutputLocation;
	std::wstring strAppLocation; 

	// lets get the app location
	strAppLocation = getAppLocation();
	strOutputLocation = strAppLocation += L"\\";
	strOutputLocation += strOutputName;
	
	// find location of the resource and get handle to it
	hRes = FindResource( NULL, MAKEINTRESOURCE(nResourceId), strCustomResName.c_str() );
	
	// loads the specified resource into global memory. 
	hResourceLoaded = LoadResource( NULL, hRes ); 

	// get a pointer to the loaded resource!
	lpResLock = (char*)LockResource( hResourceLoaded ); 

	// determine the size of the resource, so we know how much to write out to file!  
	dwSizeRes = SizeofResource( NULL, hRes );

	std::ofstream outputFile(strOutputLocation.c_str(), std::ios::binary);

	outputFile.write((const char*)lpResLock, dwSizeRes);
	outputFile.close();
}

void BinRes::ExtractBinResource(std::wstring strCustomResName, int nResourceId, std::wstring strOutputName, std::wstring path)
{
	HGLOBAL hResourceLoaded;		// handle to loaded resource 
	HRSRC hRes;						// handle/ptr. to res. info. 
	char* lpResLock;				// pointer to resource data 
	DWORD dwSizeRes;
	std::wstring strOutputLocation;
	std::wstring strAppLocation;

	strOutputLocation = path += L"\\";
	strOutputLocation += strOutputName;

	// find location of the resource and get handle to it
	hRes = FindResource(NULL, MAKEINTRESOURCE(nResourceId), strCustomResName.c_str());

	// loads the specified resource into global memory. 
	hResourceLoaded = LoadResource(NULL, hRes);

	// get a pointer to the loaded resource!
	lpResLock = (char*)LockResource(hResourceLoaded);

	// determine the size of the resource, so we know how much to write out to file!  
	dwSizeRes = SizeofResource(NULL, hRes);

	std::ofstream outputFile(strOutputLocation.c_str(), std::ios::binary);

	outputFile.write((const char*)lpResLock, dwSizeRes);
	outputFile.close();
}

// retrieves the full path and file name for our executable file 
std::wstring BinRes::getAppLocation()
{
	TCHAR szPathName[128];
	std::wstring strPath;

	GetModuleFileName(NULL, szPathName, 128);

	strPath = szPathName;
	
	int slashPos = strPath.rfind('\\');

	if(slashPos == strPath.npos)
		throw "Unable to get exe location";

	strPath = strPath.substr(0, slashPos);
	
	return strPath;
}


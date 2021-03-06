/***** 3次元点群の出力保存 *****/


/*** インクルードファイル ***/

/* PCL 1.6.0 */
#include <pcl/io/pcd_io.h>		// PCD データ入出力
#include <pcl/io/ply_io.h>		// PLY データ入出力
#include <pcl/io/vtk_io.h>		// VTK データ入出力
#include <pcl/common/io.h>		// 入出力
#include <pcl/point_types.h>	// 点群データ型

/* C++ */
#include <iostream>

/* 作成ヘッダファイル */
#include "FunctionDefinition.h"	// 関数の定義


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** 3次元点群の PCDData 出力保存( PointXYZRGB ) ***/
int savePointCloudRGBtoPCD( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, const char* outputFileName ){
	
	/* 点群( PCD データ )の書き込み */
	cout << ">>> PCD データ( PointXYZRGB )の出力保存中" << "\r";
	if( pcl::io::savePCDFileASCII( outputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD データの書き込み失敗" << endl; return 1; }
	cout << ">>> PCD データ( PointXYZRGB )の出力保存 OK" << endl;

	return 0;
}


/*** 3次元点群の PCDData 出力保存( PointXYZRGB ) ***/
int savePointCloudtoPCD( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, const char* outputFileName ){
	
	/* 点群( PCD データ )の書き込み */
	cout << ">>> PCD データ( PointXYZ )の出力保存中" << "\r";
	if( pcl::io::savePCDFileASCII( outputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD データの書き込み失敗" << endl; return 1; }
	cout << ">>> PCD データ( PointXYZ )の出力保存 OK" << endl;

	return 0;
}


/*** 3次元点群の PLYData 出力保存( PointXYZRGB ) ***/
int savePointCloudRGBtoPLY( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, const char* outputFileName ){
	
	/* 点群( PLY データ )の書き込み */
	cout << ">>> PLY データ( PointXYZRGB )の出力保存中" << "\r";
	if( pcl::io::savePLYFileASCII( outputFileName, *cloud ) == -1 ){ cout << "Caution!: PLY データの書き込み失敗" << endl; return 1; }
	cout << ">>> PLY データ( PointXYZRGB )の出力保存 OK" << endl;

	return 0;
}


/*** 3次元点群の PLYData 出力保存( PointXYZ ) ***/
int savePointCloudtoPLY( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, const char* outputFileName ){
	
	/* 点群( PLY データ )の書き込み */
	cout << ">>> PLY データ( PointXYZ )の出力保存中" << "\r";
	if( pcl::io::savePLYFileASCII( outputFileName, *cloud ) == -1 ){ cout << "Caution!: PLY データの書き込み失敗" << endl; return 1; }
	cout << ">>> PLY データ( PointXYZ )の出力保存 OK" << endl;

	return 0;
}
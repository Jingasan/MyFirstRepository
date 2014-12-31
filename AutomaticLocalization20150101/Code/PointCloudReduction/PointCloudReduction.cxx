/***** 3次元点群数の削減 *****/


/*** インクルードファイル ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>		// 点群のデータ型
#include <pcl/filters/voxel_grid.h>	// ボクセルグリッドによる点群の間引き

/* 作成ヘッダファイル */
#include "FunctionDefinition.h"	// 関数の定義


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** 点群数の削減関数( PointXYZRGB ) ***/
int reductPointCloudRGB(
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud,
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr reCloud,
	float boxelGridSizeX,
	float boxelGridSizeY,
	float boxelGridSizeZ
){

	/* ボクセルグリッドによるダウンサンプリング */
	cout << ">>> 点群数の削減中" << "\r";
	pcl::VoxelGrid< pcl::PointXYZRGB > bg;
	bg.setInputCloud( cloud );	// 削減前の3次元点群
	bg.setLeafSize( boxelGridSizeX, boxelGridSizeY, boxelGridSizeZ );
	bg.filter( *reCloud );		// 削減後の3次元点群
	cout << ">>> 点群数の削減 OK" << endl;

	return 0;
}

/*** 点群数の削減関数( PointXYZ ) ***/
int reductPointCloud(
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud,
	pcl::PointCloud< pcl::PointXYZ >::Ptr reCloud,
	float boxelGridSizeX,
	float boxelGridSizeY,
	float boxelGridSizeZ
){

	/* ボクセルグリッドによるダウンサンプリング */
	cout << ">>> 点群数の削減中" << "\r";
	pcl::VoxelGrid< pcl::PointXYZ > bg;
	bg.setInputCloud( cloud );	// 削減前の3次元点群
	bg.setLeafSize( boxelGridSizeX, boxelGridSizeY, boxelGridSizeZ );
	bg.filter( *reCloud );		// 削減後の3次元点群
	cout << ">>> 点群数の削減 OK" << endl;

	return 0;
}
/***** 3次元点群の表示 *****/

// 【 Viewer の操作方法 】
// ・左ドラッグ - 視点の回転
// ・Shift + 左ドラッグ - 視点の平行移動
// ・Ctrl + 左ドラッグ - 画面上の回転
// ・右ドラッグ - ズーム
// ・g：メジャーの表示
// ・j：スクリーンショットの保存


/*** インクルードファイル ***/

/* PCL 1.6.0 */
#include <pcl/visualization/cloud_viewer.h>	// 点群の可視化

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 3次元点群の Viewer 表示( PointXYZRGB ) ***/
int showPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud ){
	
	/* 点群の Viewer 表示 */
	pcl::visualization::CloudViewer viewer( "PointCloud XYZRGB" );	// Viewer 名の設定
	viewer.showCloud( cloud );		// 点群の表示
	while( !viewer.wasStopped() ){}	// Viewer が閉じられるまで無限ループ	

	return 0;
}


/*** 3次元点群の Viewer 表示( PointXYZ ) ***/
int showPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud ){
	
	/* 点群の Viewer 表示 */
	pcl::visualization::CloudViewer viewer( "PointCloud XYZ" );	// Viewer 名の設定
	viewer.showCloud( cloud );		// 点群の表示
	while( !viewer.wasStopped() ){}	// Viewer が閉じられるまで無限ループ 

	return 0;
}


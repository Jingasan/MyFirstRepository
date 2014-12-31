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
#include <pcl/point_types.h>				// 点群のデータ型
#include <pcl/visualization/cloud_viewer.h>	// 点群の可視化

/* 作成ヘッダファイル */
#include "FunctionDefinition.h"	// 関数の定義


/*** 3次元点群の Viewer 表示( PointXYZRGB ) ***/
int showPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, int i ){
	
	/* Viewer 名の設定 */
	char ViewerName[ 128 ];
	sprintf( ViewerName, "PointCloud XYZRGB ID:%d", i );

	/* 点群の Viewer 表示 */
	pcl::visualization::CloudViewer viewer( ViewerName );	// Viewer 名の設定
	viewer.showCloud( cloud );		// 点群の表示
	while( !viewer.wasStopped() ){}	// Viewer が閉じられるまで無限ループ	

	return 0;
}


/*** 3次元点群の Viewer 表示( PointXYZ ) ***/
int showPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, int i ){
	
	/* Viewer 名の設定 */
	char ViewerName[ 128 ];
	sprintf( ViewerName, "PointCloud XYZ ID:%d", i );

	/* 点群の Viewer 表示 */
	pcl::visualization::CloudViewer viewer( ViewerName );	// Viewer 名の設定
	viewer.showCloud( cloud );		// 点群の表示
	while( !viewer.wasStopped() ){}	// Viewer が閉じられるまで無限ループ 

	return 0;
}
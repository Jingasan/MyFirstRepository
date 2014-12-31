/***** Levenberg Marquardt 法を用いた最適化 *****/


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** Levenberg Marquardt 法を用いた最適化 ***/
int ICPOptomization(
	int** DistanceField,							// DF
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,	// モデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,	// 探索対象点群
	Eigen::Matrix< double, 3, 3 > *R,				// 回転行列
	Eigen::Vector3d *t								// 並進ベクトル
){
	
	/*** 変数の宣言 ***/

	


	return 0;
}
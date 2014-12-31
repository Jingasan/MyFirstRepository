/***** Levenberg Marquardt 法を用いた最適化 *****/


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** Levenberg Marquardt 法を用いた最適化 ***/
int LevMarOptimization(
	int** DistanceField,							// DF
	int** ClosestPointID,							// 最近点の ID
	struct ModelInformation* targetInformation,		// 探索対象点群の各種情報
	struct ModelInformation* modelInformation,		// モデル点群の各種情報
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,	// モデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,	// 探索対象点群
	vector< Eigen::Matrix< double, 3, 3 > > *ER,	// 全探索結果の回転行列
	vector< Eigen::Vector3d > *Et,					// 全探索結果の並進ベクトル
	vector< double > *DFval							// 全探索結果の DF 値
){
	

	/*** Marquart法の前処理：初期位置姿勢 ***/

	/* 点群のメモリ確保 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr targetPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );		// 重心変換後の探索対象点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr modelPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );		// 重心変換後のモデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// 全探索結果によるアフィン変換後のモデル点群
	pcl::PointXYZRGB temp;																						// 点座標の一時格納用変数

	/* 探索対象点群 */
	for( int i = 0; i < cloudT->size(); i++ ){
			
		/* 探索対象点群を DF 座標系へ変換：DF 座標系は点群の重心が原点 */
		temp.x = cloudT->points[ i ].x - ( targetInformation->gravX / (double)TOMETER );
		temp.y = cloudT->points[ i ].y - ( targetInformation->gravY / (double)TOMETER );
		temp.z = cloudT->points[ i ].z - ( targetInformation->gravZ / (double)TOMETER );
		temp.r = 0;
		temp.g = 255;
		temp.b = 0;
		targetPointCloud->points.push_back( temp );
	}
	targetPointCloud->width = targetPointCloud->points.size();
	targetPointCloud->height = 1;

	/* モデル点群 */
	for( int i = 0; i < cloudM->size(); i++ ){
			
		/* モデル点群を DF 座標系へ変換：DF 座標系は点群の重心が原点 */
		temp.x = cloudM->points[ i ].x - ( modelInformation->gravX / (double)TOMETER );
		temp.y = cloudM->points[ i ].y - ( modelInformation->gravY / (double)TOMETER );
		temp.z = cloudM->points[ i ].z - ( modelInformation->gravZ / (double)TOMETER );
		temp.r = 255;
		temp.g = 0;
		temp.b = 0;
		modelPointCloud->points.push_back( temp );
	}
	modelPointCloud->width = cloudM->points.size();
	modelPointCloud->height = 1;

	/* 全探索結果によるアフィン変換 */
	Eigen::Vector3d p;	// モデル点群の一時格納用配列
	Eigen::Vector3d ap;	// アフィン変換後点群の一時格納用配列
	for( int i = 0; i < cloudM->size(); i++ ){
		
		/* 全探索結果によるアフィン変換 */
		p( 0 ) = modelPointCloud->points[ i ].x;
		p( 1 ) = modelPointCloud->points[ i ].y;
		p( 2 ) = modelPointCloud->points[ i ].z;
		ap = (*ER)[ 0 ] * p + (*Et)[ 0 ];
		temp.x = ap( 0 );
		temp.y = ap( 1 );
		temp.z = ap( 2 );
		temp.r = modelPointCloud->points[ i ].r;
		temp.g = modelPointCloud->points[ i ].g;
		temp.b = modelPointCloud->points[ i ].b;
		transformedPointCloud->points.push_back( temp );
	}
	transformedPointCloud->width = modelPointCloud->points.size();
	transformedPointCloud->height = 1;


	/*** 変数の宣言 ***/
	Eigen::Matrix< double, 3, 3 > Rtemp;	// Rodrigues 変換前の 3 * 3 回転行列
	Eigen::Vector3d RodriguesVector;		// Rodrigues を用いた回転表現 r = [ r1, r2, r3 ]
	Eigen::Matrix< double, 3, 3 > dR[ 3 ];

	/*** Rodrigues を用いた回転表現への変換 ***/
	
	/* Rodrigues 変換前の回転行列 */
	Rtemp = (*ER)[ 0 ];

	/* Rodrigues の回転表現 r = [ r1, r2, r3 ] への変換 */
	ConvertRodrigues( &Rtemp, &RodriguesVector );

	/*** 回転行列の r1, r2, r3 成分による偏微分 ***/
	dR[ 0 ] = PartialDerivativeForRodrigues( 0, &RodriguesVector );	// r1 による偏微分結果の 3 * 3 回転行列
	dR[ 1 ] = PartialDerivativeForRodrigues( 1, &RodriguesVector );	// r2 による偏微分結果の 3 * 3 回転行列
	dR[ 2 ] = PartialDerivativeForRodrigues( 2, &RodriguesVector );	// r3 による偏微分結果の 3 * 3 回転行列




#ifdef COMMENT

	/* Rodrigues の回転表現により構成される行列 [r]x の作成 */
	Eigen::Matrix< double, 3, 3 > RodriguesMatrix;	// Rodrigues の回転表現により構成される 3 * 3 行列 [r]x
	RodriguesMatrix( 0, 0 ) = 0; RodriguesMatrix( 0, 1 ) = - RodriguesVector( 2 ); RodriguesMatrix( 0, 2 ) = RodriguesVector( 1 );
	RodriguesMatrix( 1, 0 ) = RodriguesVector( 2 ); RodriguesMatrix( 1, 1 ) = 0; RodriguesMatrix( 1, 2 ) = - RodriguesVector( 0 );
	RodriguesMatrix( 2, 0 ) = - RodriguesVector( 1 ); RodriguesMatrix( 2, 1 ) = RodriguesVector( 0 ); RodriguesMatrix( 2, 2 ) = 0;
	
	cout << endl << ">> Rodrigues 変換前の回転行列 R" << endl;
	cout << Rtemp << endl;
	cout << endl << ">> Rodrigues を用いた回転表現 r = [ r1, r2, r3 ]" << endl;
	cout << "r1: " << RodriguesVector( 0 ) << ", r2: " << RodriguesVector( 1 ) << ", r3: " << RodriguesVector( 2 ) << endl;
	cout << ">> Rodrigues を用いた回転表現により構成される行列 [r]x" << endl;
	cout << RodriguesMatrix << endl;
	cout << endl << ">> r1 による偏微分結果の回転行列" << endl;
	cout << dR[ 0 ] << endl;
	cout << endl << ">> r2 による偏微分結果の回転行列" << endl;
	cout << dR[ 1 ] << endl;
	cout << endl << ">> r3 による偏微分結果の回転行列" << endl;
	cout << dR[ 2 ] << endl;

#endif





#ifdef COMMENT
	cout << endl << "--- 全探索結果 ---" << endl;
	cout << "R:" << endl << (*ER)[ 0 ]( 0, 0 ) << " " << (*ER)[ 0 ]( 0, 1 ) << " " << (*ER)[ 0 ]( 0, 2 ) << endl;
	cout << (*ER)[ 0 ]( 1, 0 ) << " " << (*ER)[ 0 ]( 1, 1 ) << " " << (*ER)[ 0 ]( 1, 2 ) << endl;
	cout << (*ER)[ 0 ]( 2, 0 ) << " " << (*ER)[ 0 ]( 2, 1 ) << " " << (*ER)[ 0 ]( 2, 2 ) << endl;
	cout << "t:" << endl << (*Et)[ 0 ]( 0 ) << " "<< (*Et)[ 0 ]( 1 ) << " "<< (*Et)[ 0 ]( 2 ) << endl;
	cout << "DF:" << endl << (*DFval)[ 0 ] << endl << endl;
#endif


	return 0;
}
/***** ICP アルゴリズムを用いた最適化 *****/


/*** インクルードファイル ***/

/* PCL 1.6.0 */
#include <pcl/registration/transformation_estimation.h>	// 点群の座標変換
#include <pcl/registration/icp.h>						// ICP

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** ICP を用いた最適化( PointXYZRGB ) ***/
int ICPOptimizationRGB(
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudM,		// モデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudT,		// 探索対象点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr newCloudM,		// 位置合わせ後のモデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedCloud,	// ICP 後の統合点群
	struct ModelInformation* targetInformation,				// 探索対象点群の各種情報
	struct ModelInformation* modelInformation,				// モデル点群の各種情報
	vector< Eigen::Matrix< double, 3, 3 > > *ER,			// 全探索結果の回転行列
	vector< Eigen::Vector3d > *Et							// 全探索結果の並進ベクトル
){

	/*** ICP の前処理：初期位置姿勢 ***/

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
		temp.r = cloudT->points[ i ].r;
		temp.g = cloudT->points[ i ].g;
		temp.b = cloudT->points[ i ].b;
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
		temp.r = cloudM->points[ i ].r;
		temp.g = cloudM->points[ i ].g;
		temp.b = cloudM->points[ i ].b;
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


	/*** ICP アルゴリズム ***/
	cout << endl << "----- ICP による最適化開始 -----" << "\r";
	pcl::IterativeClosestPoint< pcl::PointXYZRGB, pcl::PointXYZRGB > icp;
	icp.setInputCloud( transformedPointCloud );	// 位置合わせ対象の点群
	icp.setInputTarget( targetPointCloud );	// 位置合わせ先の点群
	icp.setRANSACOutlierRejectionThreshold( RANSAC_OUTLIER_REJECTION_THRESHOLD );	// RANSACを用いた外れ値除去の閾値設定
	//icp.setMaxCorrespondenceDistance( MAX_CORRESPONDENCE_DISTANCE );	// 対応距離の最大値
	icp.setMaximumIterations( ICP_LOOP_MAX );	// ICPの最大反復回数
	icp.align( *newCloudM );					// 位置合わせ後の点群
	cout << "----- ICP による最適化終了 -----" << endl;


#ifdef ICPRESULT
	
	/* ICP 後の点群の統合 */
	*mergedCloud = *targetPointCloud + *newCloudM;

	/* ICP の結果 */
	Eigen::Matrix4f Rt = icp.getFinalTransformation();	// R, t の 4 * 4 行列
	double icpRMSE = sqrt( newCloudM->size() * icp.getFitnessScore() ) / newCloudM->size();		// RMSE 評価値

	/* PCL を用いた ICP 結果 R, t と RMSE のコマンドライン出力 */
	cout << endl << "----- ICP 最適化結果 [ R | t ] ----" << endl;
	cout << icp.getFinalTransformation() << endl;
	cout << "1点間あたりの RMSE : " << icpRMSE << endl;

	/* ICP 結果の出力保存 */
	FILE *outputFp1, *outputFp2;
	if( ( outputFp1 = fopen( IcpRtFileName, "w" ) ) == NULL ){ cout << "Caution!: " << IcpRtFileName << " open error" << endl; return 1; }
	if( ( outputFp2 = fopen( IcpRMSEFileName, "w" ) ) == NULL ){ cout << "Caution!: " << IcpRMSEFileName << " open error" << endl; return 1; }
	fprintf( outputFp1, "%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
		Rt( 0, 0 ), Rt( 0, 1 ), Rt( 0, 2 ), Rt( 0, 3 ), Rt( 1, 0 ), Rt( 1, 1 ), Rt( 1, 2 ), Rt( 1, 3 ),
		Rt( 2, 0 ), Rt( 2, 1 ), Rt( 2, 2 ), Rt( 2, 3 ), Rt( 3, 0 ), Rt( 3, 1 ), Rt( 3, 2 ), Rt( 3, 3 )
	);
	fprintf( outputFp2, "RMSE,%f", icpRMSE );
	fclose( outputFp1 ); fclose( outputFp2 );

	/* 点群の出力保存 */
	savePointCloudRGBtoPCD( mergedCloud, IcpPCDFileName );
	savePointCloudRGBtoPLY( mergedCloud, IcpPLYFileName );

#endif

#ifdef VISUALIZE
	
	/* 統合された点群の可視化 */
	showPointCloudRGB( mergedCloud );

#endif

	return 0;
}


/*** ICP を用いた最適化( PointXYZ ) ***/
int ICPOptimization(
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,			// モデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,			// 探索対象点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr newCloudM,		// 位置合わせ後のモデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedCloud,	// ICP 後の統合点群
	struct ModelInformation* targetInformation,				// 探索対象点群の各種情報
	struct ModelInformation* modelInformation,				// モデル点群の各種情報
	vector< Eigen::Matrix< double, 3, 3 > > *ER,			// 全探索結果の回転行列
	vector< Eigen::Vector3d > *Et							// 全探索結果の並進ベクトル
){
	
	/*** ICP の前処理：初期位置姿勢 ***/

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


	/*** ICP アルゴリズム ***/
	cout << "----- ICP による最適化開始 -----" << "\r";
	pcl::IterativeClosestPoint< pcl::PointXYZRGB, pcl::PointXYZRGB > icp;
	icp.setInputCloud( transformedPointCloud );	// 位置合わせ対象の点群
	icp.setInputTarget( targetPointCloud );		// 位置合わせ先の点群
	icp.setRANSACOutlierRejectionThreshold( RANSAC_OUTLIER_REJECTION_THRESHOLD );	// RANSACを用いた外れ値除去の閾値設定
	//icp.setMaxCorrespondenceDistance( MAX_CORRESPONDENCE_DISTANCE );	// 対応距離の最大値
	icp.setMaximumIterations( ICP_LOOP_MAX );	// ICPの最大反復回数
	icp.align( *newCloudM );					// 位置合わせ後の点群
	cout << "----- ICP による最適化終了 -----" << endl;


#ifdef ICPRESULT
	
	/* ICP 後の点群の統合 */
	*mergedCloud = *targetPointCloud + *newCloudM;

	/* ICP の結果 */
	Eigen::Matrix4f Rt = icp.getFinalTransformation();	// R, t の 4 * 4 行列
	double icpRMSE = sqrt( newCloudM->size() * icp.getFitnessScore() ) / newCloudM->size();		// RMSE 評価値

	
	/* PCL を用いた ICP 結果 R, t と RMSE のコマンドライン出力 */
	cout << endl << "----- ICP 最適化結果 [ R | t ] ----" << endl;
	cout << icp.getFinalTransformation() << endl;
	cout << "1点間あたりの RMSE : " << icpRMSE << "[mm]" << endl;

	/* ICP 結果の出力保存 */
	FILE *outputFp1, *outputFp2;
	if( ( outputFp1 = fopen( IcpRtFileName, "w" ) ) == NULL ){ cout << "Caution!: " << IcpRtFileName << " open error" << endl; return 1; }
	if( ( outputFp2 = fopen( IcpRMSEFileName, "w" ) ) == NULL ){ cout << "Caution!: " << IcpRMSEFileName << " open error" << endl; return 1; }
	fprintf( outputFp1, "%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
		Rt( 0, 0 ), Rt( 0, 1 ), Rt( 0, 2 ), Rt( 0, 3 ), Rt( 1, 0 ), Rt( 1, 1 ), Rt( 1, 2 ), Rt( 1, 3 ),
		Rt( 2, 0 ), Rt( 2, 1 ), Rt( 2, 2 ), Rt( 2, 3 ), Rt( 3, 0 ), Rt( 3, 1 ), Rt( 3, 2 ), Rt( 3, 3 )
	);
	fprintf( outputFp2, "RMSE,%f", icpRMSE );
	fclose( outputFp1 ); fclose( outputFp2 );

	/* 点群の出力保存 */
	savePointCloudRGBtoPCD( mergedCloud, IcpPCDFileName );
	savePointCloudRGBtoPLY( mergedCloud, IcpPLYFileName );

#endif

#ifdef VISUALIZE
	
	/* 統合された点群の可視化 */
	showPointCloudRGB( mergedCloud );

#endif

	return 0;
}
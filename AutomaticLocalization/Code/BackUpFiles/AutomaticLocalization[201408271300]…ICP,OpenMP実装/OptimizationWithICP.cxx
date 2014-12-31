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
int ICPOptomizationRGB(
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudM,		// モデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudT,		// 探索対象点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr newCloudM,		// 位置合わせ後のモデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedCloud	// ICP 後の統合点群
	//Eigen::Matrix< double, 3, 3 > *RIcp,					// 回転行列
	//Eigen::Vector3d *tIcp									// 並進ベクトル
){

	/* ICP アルゴリズム */
	cout << "----- ICP による最適化開始 -----" << "\r";
	pcl::IterativeClosestPoint< pcl::PointXYZRGB, pcl::PointXYZRGB > icp;
	icp.setInputCloud( cloudM );					// 位置合わせ対象の点群
	icp.setInputTarget( cloudT );					// 位置合わせ先の点群
	icp.setRANSACOutlierRejectionThreshold( 1.0 );	// RANSACを用いた外れ値除去の閾値設定
	//icp.setMaxCorrespondenceDistance( 0.3 );		// 対応距離の最大値
	icp.setMaximumIterations( ICP_LOOP_MAX );		// ICPの最大反復回数
	icp.align( *newCloudM );						// 位置合わせ後の点群
	cout << "----- ICP による最適化終了 -----" << endl;


#ifdef ICPRESULT
	
	/* ICP 後の点群の統合 */
	*mergedCloud = *cloudT + *newCloudM;

	/* ICP の結果 */
	Eigen::Matrix4f Rt = icp.getFinalTransformation();	// R, t の 4 * 4 行列
	double icpRMSE = sqrt( icp.getFitnessScore() );		// RMSE 評価値

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
int ICPOptomization(
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,		// モデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,		// 探索対象点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr newCloudM,	// 位置合わせ後のモデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr mergedCloud	// ICP 後の統合点群
	//Eigen::Matrix< double, 3, 3 > *RIcp,				// 回転行列
	//Eigen::Vector3d *tIcp								// 並進ベクトル
){
	
	/* ICP アルゴリズム */
	cout << "----- ICP による最適化開始 -----" << "\r";
	pcl::IterativeClosestPoint< pcl::PointXYZ, pcl::PointXYZ > icp;
	icp.setInputCloud( cloudM );					// 位置合わせ対象の点群
	icp.setInputTarget( cloudT );					// 位置合わせ先の点群
	icp.setRANSACOutlierRejectionThreshold( 1.0 );	// RANSACを用いた外れ値除去の閾値設定
	//icp.setMaxCorrespondenceDistance( 0.3 );		// 対応距離の最大値
	icp.setMaximumIterations( ICP_LOOP_MAX );		// ICPの最大反復回数
	icp.align( *newCloudM );						// 位置合わせ後の点群
	cout << "----- ICP による最適化終了 -----" << endl;


#ifdef ICPRESULT
	
	/* ICP 後の点群の統合 */
	*mergedCloud = *cloudT + *newCloudM;

	/* ICP の結果 */
	Eigen::Matrix4f Rt = icp.getFinalTransformation();	// R, t の 4 * 4 行列
	double icpRMSE = sqrt( icp.getFitnessScore() );		// RMSE 評価値
	
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
	savePointCloudtoPCD( mergedCloud, IcpPCDFileName );
	savePointCloudtoPLY( mergedCloud, IcpPLYFileName );

#endif

#ifdef VISUALIZE
	
	/* 統合された点群の可視化 */
	showPointCloud( mergedCloud );

#endif

	return 0;
}
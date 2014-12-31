/***** 全探索 *****/


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** 全探索 ***/
int ExhaustiveSearch(
	int** DistanceField,							// DF
	struct ModelInformation* targetInformation,		// 探索対象点群の各種情報
	struct ModelInformation* modelInformation,		// モデル点群の各種情報
	int* minSumofDistance,							// 探索対象点群とモデル点群との距離評価合計値の最小値
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,	// モデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,	// 探索対象点群
	vector< Eigen::Matrix< double, 3, 3 > > *ER,		// 全探索結果の回転行列
	vector< Eigen::Vector3d > *Et,					// 全探索結果の並進ベクトル
	vector< double > *DFval							// 全探索結果の DF 値
	//Eigen::Matrix< double, 3, 3 > **R				// 回転行列 R
){
	
	/*** 変数の宣言 ***/

	/* 全探索関連 */
	FILE *inputFp1;	// サンプリングされた四元数の入力ファイル
	FILE *inputFp2;	// サンプリングされた四元数と対応する回転角θΦΨの入力ファイル
	vector< vector< float > > quaternions;	// サンプリングされた四元数
	vector< float > quaternion;				// 四元数
	vector< vector< float > > degrees;		// サンプリングされた回転角θΦΨ
	vector< float > degree;					// 回転角θΦΨ
	int sampX = SAMP_X;	// X 方向の並進のサンプリング幅
	int sampY = SAMP_Y;	// Y 方向の並進のサンプリング幅
	int sampZ = SAMP_Z;	// Z 方向の並進のサンプリング幅
	int sampRLevel = SAMP_R_LEVEL;	// 回転空間の均等なサンプリングの解像度レベル ⇒ 0, 1, 2, 3 から指定
	int lowerX = - targetInformation->deltaX / ( 2 * TOMETER );	// X 方向の並進の探索範囲の下限
	int lowerY = - targetInformation->deltaY / ( 2 * TOMETER );	// Y 方向の並進の探索範囲の下限
	int lowerZ = - targetInformation->deltaZ / ( 2 * TOMETER );	// Z 方向の並進の探索範囲の下限
	int upperX = targetInformation->deltaX / ( 2 * TOMETER );	// X 方向の並進の探索範囲の上限
	int upperY = targetInformation->deltaY / ( 2 * TOMETER );	// Y 方向の並進の探索範囲の上限
	int upperZ = targetInformation->deltaZ / ( 2 * TOMETER );	// Z 方向の並進の探索範囲の上限
	lowerX = lowerX / LIMIT_X;	// X 方向の並進の探索範囲の下限値の絞り込み
	lowerY = lowerY / LIMIT_Y;	// Y 方向の並進の探索範囲の下限値の絞り込み 
	lowerZ = lowerZ / LIMIT_Z;	// Z 方向の並進の探索範囲の下限値の絞り込み
	upperX = upperX / LIMIT_X;	// X 方向の並進の探索範囲の上限値の絞り込み
	upperY = upperY / LIMIT_Y;	// Y 方向の並進の探索範囲の上限値の絞り込み
	upperZ = upperZ / LIMIT_Z;	// Z 方向の並進の探索範囲の上限値の絞り込み
	int SumofDistance;			// DF を用いた探索対象点群とモデル点群との距離評価値
	double distanceMeans;		// 1点当たりの距離評価値

	/* 探索対象点群とモデル点群との距離評価関連 */
	int iSize = targetInformation->iSize;			// DF の i 方向のサイズ
	int jSize = targetInformation->jSize;			// DF の j 方向のサイズ
	int kSize = targetInformation->kSize;			// DF の k 方向のサイズ
	int ijSize = iSize * jSize;						// DF の i, j 方向の面積
	double targetXSize = targetInformation->deltaX;	// 探索対象点群の X 方向の最大幅 
	double targetYSize = targetInformation->deltaY;	// 探索対象点群の Y 方向の最大幅
	double targetZSize = targetInformation->deltaZ;	// 探索対象点群の Z 方向の最大幅
	double gravX = targetInformation->gravX;	// 探索対象点群の重心 X
	double gravY = targetInformation->gravY;	// 探索対象点群の重心 Y
	double gravZ = targetInformation->gravZ;	// 探索対象点群の重心 Z
	double mGravX;	// モデル点群の重心 X
	double mGravY;	// モデル点群の重心 Y
	double mGravZ;	// モデル点群の重心 Z
	int sum = 0;	// 探索対象点群とモデル点群との距離評価値の合計

	int minRID;
	Eigen::Matrix< double, 3, 3 > minR;
	Eigen::Vector3d minDegree;
	Eigen::Vector3d mint;
	Eigen::Matrix< double, 3, 3 > *R;	// 均等にサンプリングされた回転行列
	Eigen::Vector3d t;					// 並進ベクトル
	Eigen::Vector3d p;					// 回転前点群の一時格納用配列
	Eigen::Vector3d rp;					// 回転後点群の一時格納用配列

	/* 初期化 */
	modelInformation->Xmin = INFINITY;
	modelInformation->Ymin = INFINITY;
	modelInformation->Zmin = INFINITY;
	modelInformation->Xmax = -INFINITY;
	modelInformation->Ymax = -INFINITY;
	modelInformation->Zmax = -INFINITY;

	/* メモリの確保 */
	quaternions.resize( 0 );
	quaternion.resize( 4 );
	degrees.resize( 0 );
	degree.resize( 3 );


	/*** モデル点群の重心の算出 ***/

	/* 点群の端点を保存 */
	for( size_t i = 0; i < cloudM->points.size(); i++ ){
		
		/* X, Y, Z の最小値を設定 */
		if( cloudM->points[ i ].x * TOMETER < modelInformation->Xmin ) modelInformation->Xmin = cloudM->points[ i ].x * TOMETER;
		if( cloudM->points[ i ].y * TOMETER < modelInformation->Ymin ) modelInformation->Ymin = cloudM->points[ i ].y * TOMETER;
		if( cloudM->points[ i ].z * TOMETER < modelInformation->Zmin ) modelInformation->Zmin = cloudM->points[ i ].z * TOMETER;
		
		/* X, Y, Z の最大値を設定 */
		if( cloudM->points[ i ].x * TOMETER > modelInformation->Xmax ) modelInformation->Xmax = cloudM->points[ i ].x * TOMETER;
		if( cloudM->points[ i ].y * TOMETER > modelInformation->Ymax ) modelInformation->Ymax = cloudM->points[ i ].y * TOMETER;
		if( cloudM->points[ i ].z * TOMETER > modelInformation->Zmax ) modelInformation->Zmax = cloudM->points[ i ].z * TOMETER;
	}

	/* 点群の重心の算出 */ 
	mGravX = ( modelInformation->Xmax + modelInformation->Xmin ) / 2;
	mGravY = ( modelInformation->Ymax + modelInformation->Ymin ) / 2;
	mGravZ = ( modelInformation->Zmax + modelInformation->Zmin ) / 2;

	/* 点群の重心をモデル情報として格納 */
	modelInformation->gravX = mGravX;
	modelInformation->gravY = mGravY;
	modelInformation->gravZ = mGravZ;

	/* 点群の重心を原点に設定 */
	modelInformation->Xmin -= mGravX; modelInformation->Xmax -= mGravX;
	modelInformation->Ymin -= mGravY; modelInformation->Ymax -= mGravY;
	modelInformation->Zmin -= mGravZ; modelInformation->Zmax -= mGravZ;
	
	/* 点群の最大幅の算出 */
	modelInformation->deltaX = modelInformation->Xmax - modelInformation->Xmin;
	modelInformation->deltaY = modelInformation->Ymax - modelInformation->Ymin;
	modelInformation->deltaZ = modelInformation->Zmax - modelInformation->Zmin;
    
	/* 最も大きい値を持つ点群幅の算出 X方向 or Y方向 or Z方向 */
	if( modelInformation->deltaX > modelInformation->deltaY ){
		if( modelInformation->deltaX > modelInformation->deltaZ ){
			modelInformation->max = modelInformation->deltaX;
		}else{
			modelInformation->max = modelInformation->deltaZ;
		}
	}else{
		if( modelInformation->deltaY > modelInformation->deltaZ ){
			modelInformation->max = modelInformation->deltaY;
		}else{
			modelInformation->max = modelInformation->deltaZ;
		}
	}


	/*** サンプリングされた四元数とその回転角の読み込みと回転行列への変換 ***/

	/* ファイルの展開 */
	if( sampRLevel == 0 ){
		if( ( inputFp1 = fopen( QuaternionFileName1, "r" ) ) == NULL ){ cout << "Caution!: " << QuaternionFileName1 << " open error" << endl; return 1; }
		if( ( inputFp2 = fopen( DegreeFileName1, "r" ) ) == NULL ){ cout << "Caution!: " << DegreeFileName1 << " open error" << endl; return 1; }
	}else if( sampRLevel == 1 ){
		if( ( inputFp1 = fopen( QuaternionFileName2, "r" ) ) == NULL ){ cout << "Caution!: " << QuaternionFileName2 << " open error" << endl; return 1; }
		if( ( inputFp2 = fopen( DegreeFileName2, "r" ) ) == NULL ){ cout << "Caution!: " << DegreeFileName2 << " open error" << endl; return 1; }
	}else if( sampRLevel == 2 ){
		if( ( inputFp1 = fopen( QuaternionFileName3, "r" ) ) == NULL ){ cout << "Caution!: " << QuaternionFileName3 << " open error" << endl; return 1; }
		if( ( inputFp2 = fopen( DegreeFileName3, "r" ) ) == NULL ){ cout << "Caution!: " << DegreeFileName3 << " open error" << endl; return 1; }
	}else if( sampRLevel == 3 ){
		if( ( inputFp1 = fopen( QuaternionFileName4, "r" ) ) == NULL ){ cout << "Caution!: " << QuaternionFileName4 << " open error" << endl; return 1; }
		if( ( inputFp2 = fopen( DegreeFileName4, "r" ) ) == NULL ){ cout << "Caution!: " << DegreeFileName4 << " open error" << endl; return 1; }
	}else{ cout << "Caution!: Setting SampRLevel error" << endl; return 1; }

	/* ファイル内容の読み込み */
	while( fscanf( inputFp1, "%f,%f,%f,%f", &quaternion[ 0 ], &quaternion[ 1 ], &quaternion[ 2 ], &quaternion[ 3 ] ) != EOF ) quaternions.push_back( quaternion );	// 2次元配列の末尾への1次元配列の追加
	while( fscanf( inputFp2, "%f,%f,%f", &degree[ 0 ], &degree[ 1 ], &degree[ 2 ] ) != EOF ) degrees.push_back( degree );	// 2次元配列の末尾への1次元配列の追加
	
	/* メモリの解放 */
	fclose( inputFp1 );
	fclose( inputFp2 );

#ifdef COMMENT
	for( int i = 0; i < quaternions.size(); i++ ){
		cout << "Quaternion " << i << " : ";
		for( int j = 0; j < quaternions[ i ].size(); j++ ){ cout << quaternions[ i ][ j ] << " "; }
		cout << endl << "Degree: ";
		for( int j = 0; j < degrees[ i ].size(); j++ ){ cout << degrees[ i ][ j ] << " "; }
		cout << endl << endl;
	}
#endif

	/* 回転行列のメモリ確保 */
	R = new Eigen::Matrix< double, 3, 3 >[ quaternions.size() ];

	/* 回転行列の復元 */
	for( int i = 0; i < quaternions.size(); i++ ){

		double element0 = (double)quaternions[ i ][ 0 ];	// 四元数の1つ目の要素
		double element1 = (double)quaternions[ i ][ 1 ];	// 四元数の2つ目の要素 
		double element2 = (double)quaternions[ i ][ 2 ];	// 四元数の3つ目の要素
		double element3 = (double)quaternions[ i ][ 3 ];	// 四元数の4つ目の要素
		double element0S = pow( element0, 2 );	// 四元数の1つ目の要素の2乗
		double element1S = pow( element1, 2 );	// 四元数の2つ目の要素の2乗
		double element2S = pow( element2, 2 );	// 四元数の3つ目の要素の2乗
		double element3S = pow( element3, 2 );	// 四元数の4つ目の要素の2乗

		/* 四元数形式からの回転行列の復元 */
		R[ i ]( 0, 0 ) = element0S + element1S - element2S - element3S;		// 1行1列目の要素
		R[ i ]( 0, 1 ) = 2 * ( element1 * element2 - element0 * element3 );	// 1行2列目の要素
		R[ i ]( 0, 2 ) = 2 * ( element1 * element3 + element0 * element2 );	// 1行3列目の要素
		R[ i ]( 1, 0 ) = 2 * ( element1 * element2 + element0 * element3 );	// 2行1列目の要素
		R[ i ]( 1, 1 ) = element0S - element1S + element2S - element3S;		// 2行2列目の要素
		R[ i ]( 1, 2 ) = 2 * ( element2 * element3 - element0 * element1 );	// 2行3列目の要素
		R[ i ]( 2, 0 ) = 2 * ( element1 * element3 - element0 * element2 );	// 3行1列目の要素
		R[ i ]( 2, 1 ) = 2 * ( element2 * element3 + element0 * element1 );	// 3行2列目の要素
		R[ i ]( 2, 2 ) = element0S - element1S - element2S + element3S;		// 3行3列目の要素

#ifdef COMMENT
		cout << endl << "Index: " << i << endl << R[ i ] << endl << R[ i ].determinant() << endl;
#endif
	}


	/*** 探索対象点群とモデル点群との距離評価 ***/
	cout << "--- 探索対象点群とモデル点群との距離評価 ---" << endl;

	/* モデル点群の回転 */
	for( int r = 0; r < quaternions.size(); r++ ){

		/* 回転後のモデル点群 */
		pcl::PointCloud< pcl::PointXYZ >::Ptr rotatedPointCloud( new pcl::PointCloud< pcl::PointXYZ > );
		pcl::PointXYZ temp1;	// 点座標の一時格納用変数

		cout << "RotationID: " << r << endl << "Degree: ";
		for( int i = 0; i < degrees[ r ].size(); i++ ){ cout << degrees[ r ][ i ] << " "; }
		cout << endl << "R: " << endl << R[ r ] << endl;

		/* DF の座標系への変換, 回転後のモデル点群 */
		for( int i = 0; i < cloudM->size(); i++ ){
			
			/* モデル点群を DF 座標系へ変換：DF 座標系は点群の重心が原点 */
			p( 0 ) = cloudM->points[ i ].x - ( mGravX / (double)TOMETER );
			p( 1 ) = cloudM->points[ i ].y - ( mGravY / (double)TOMETER );
			p( 2 ) = cloudM->points[ i ].z - ( mGravZ / (double)TOMETER );
			
			/* DF の座標系原点を中心としたモデル点群の回転 */
			rp = R[ r ] * p;
			
			/* 回転後の点群の格納 */
			temp1.x = rp( 0 );
			temp1.y = rp( 1 );
			temp1.z = rp( 2 );
			//temp1.x = p( 0 );
			//temp1.y = p( 1 );
			//temp1.z = p( 2 );
			rotatedPointCloud->points.push_back( temp1 );
		}
		rotatedPointCloud->width = rotatedPointCloud->points.size();
		rotatedPointCloud->height = 1;

		
		for( int tz = lowerZ; tz < upperZ; tz += sampZ ){
			for( int ty = lowerY; ty < upperY; ty += sampY ){
				for( int tx = lowerX; tx < upperX; tx += sampX ){
					//int tx = 0; int ty = 0; int tz = 0;

					/* DF の座標系への変換, 回転並行移動後のモデル点群 */
					pcl::PointCloud< pcl::PointXYZ >::Ptr transPointCloud( new pcl::PointCloud< pcl::PointXYZ > );

					/* モデル点群の DF の座標系への変換と回転並行移動 */
					t( 0 ) = (double)tx; t( 1 ) = (double)ty; t( 2 ) = (double)tz;
					for( int i = 0; i < rotatedPointCloud->size(); i++ ){
					
						temp1.x = rotatedPointCloud->points[ i ].x + t( 0 );
						temp1.y = rotatedPointCloud->points[ i ].y + t( 1 );
						temp1.z = rotatedPointCloud->points[ i ].z + t( 2 );
						//temp1.x = ( cloudM->points[ i ].x - ( mGravX / (double)TOMETER ) ) + t( 0 );
						//temp1.y = ( cloudM->points[ i ].y - ( mGravY / (double)TOMETER ) ) + t( 1 );
						//temp1.z = ( cloudM->points[ i ].z - ( mGravZ / (double)TOMETER ) ) + t( 2 );
						transPointCloud->points.push_back( temp1 );
					}
					transPointCloud->width = transPointCloud->points.size();
					transPointCloud->height = 1;

				
					/*** 探索対象点群とモデル点群との距離評価値の算出 ***/
					sum = 0;
					for( int n = 0; n < transPointCloud->size(); n++ ){

						/* モデル点群を DF 内にある探索対象点群の座標系へ移動 */
						transPointCloud->points[ n ].x = transPointCloud->points[ n ].x * (double)TOMETER;
						transPointCloud->points[ n ].y = transPointCloud->points[ n ].y * (double)TOMETER;
						transPointCloud->points[ n ].z = transPointCloud->points[ n ].z * (double)TOMETER;

						/* モデル点群の DF 上での座標値の算出 */
						int i = (int)( ( (double)iSize * ( transPointCloud->points[ n ].x + targetXSize ) / ( 2.0 * targetXSize ) ) + 0.5 );
						int j = (int)( ( (double)jSize * ( transPointCloud->points[ n ].y + targetYSize ) / ( 2.0 * targetYSize ) ) + 0.5 );
						int k = (int)( ( (double)kSize * ( transPointCloud->points[ n ].z + targetZSize ) / ( 2.0 * targetZSize ) ) + 0.5 );

						/* モデル点群における現在参照点が DF 内にある場合 */
						if( ( ( i >= 0 ) && ( i < iSize - 1 ) ) && ( ( j >= 0 ) && ( j < jSize - 1 ) ) && ( ( k >= 0 ) && ( k < kSize - 1 ) ) ){
						//cout << "Point( " << transPointCloud->points[ n ].x << ", " << transPointCloud->points[ n ].y << ", " << transPointCloud->points[ n ].z << " ): " << endl;
						//cout << "DF( " << i << ", " << j << ", " << k << " ): " << ( *DistanceField )[ k * ijSize + j * iSize + i ] << endl;

							/* 探索対象点群とモデル点群との距離評価値の合計値を算出 */
							sum += ( *DistanceField )[ k * ijSize + j * iSize + i ];
						}
						
						/* モデル点群における現在参照点が DF 外にある場合 */
						else{
							sum += 999;	// DF 値の取得が不可能なので、仮の距離値として 999 をカウント
						}

						/* 最小合計距離評価値よりも現在の合計距離評価値の方が大きい場合 */
						if( sum > *minSumofDistance || sum > ( INT_MAX / 10 ) ) break;
					}
#ifdef COMMENTa
					cout << "DFSize: " << iSize << " " << jSize << " " << kSize << endl;
					cout << "TargetSize: " << targetXSize << " " << targetYSize << " " << targetZSize << endl;
					cout << "TargetGravity: " << gravX << " " << gravY << " " << gravZ << endl;
					cout << "ModelGravity: " << mGravX << " " << mGravY << " " << mGravZ << endl;
					cout << "ModelSize: " << modelInformation->deltaX << " " << modelInformation->deltaY << " " << modelInformation->deltaZ << endl;
					cout << "SumofDistance: " << sum << endl;
#endif

					/* 探索対象点群とモデル点群との距離評価値の合計 */
					SumofDistance = sum;

					/* 探索対象点群とモデル点群との距離評価値の最小値の更新 */
					if( SumofDistance < *minSumofDistance ){
					
						*minSumofDistance = SumofDistance; 

						/* 最小距離評価値をとる並進ベクトル */
						mint( 0 ) = tx; mint( 1 ) = ty; mint( 2 ) = tz;
						minR = R[ r ];
						minDegree( 0 ) = degrees[ r ][ 0 ];
						minDegree( 1 ) = degrees[ r ][ 1 ];
						minDegree( 2 ) = degrees[ r ][ 2 ];
						minRID = r;

						/* 1点当たりの距離評価値 */
						distanceMeans = (double)( *minSumofDistance ) / (double)cloudM->size();	// 1点あたりどのくらいずれているか
#ifdef COMMENT
						if( *minSumofDistance < 10000 ){
							cout << "最小合計距離評価値: " << *minSumofDistance << endl;
							cout << "t: " << mint( 0 ) << ", " << mint( 1 ) << ", " << mint( 2 ) << endl;
							cout << "R ID: " << minRID << endl;
							cout << "R: " << minR( 0, 0 ) << "\t" << minR( 0, 1 ) << "\t" << minR( 0, 2 ) << endl;
							cout << "   " << minR( 1, 0 ) << "\t" << minR( 1, 1 ) << "\t" << minR( 1, 2 ) << endl;
							cout << "   " << minR( 2, 0 ) << "\t" << minR( 2, 1 ) << "\t" << minR( 2, 2 ) << endl;
							cout << "Degree: " << minDegree( 0 ) << ", " << minDegree( 1 ) << ", " << minDegree( 2 ) << endl;
							cout << "DistanceMeans: " << distanceMeans << endl;
						}
#endif
					}
				}
			}
		}

#ifdef COMMENT
		cout << "最小合計距離評価値: " << *minSumofDistance << endl;
		cout << "t: " << mint( 0 ) << ", " << mint( 1 ) << ", " << mint( 2 ) << endl;
		cout << "DistanceMeans: " << distanceMeans << endl << endl;
#endif
	}


	/* 全探索結果のコマンドライン出力 */
	cout << "----- 全探索結果 -----" << endl;
	cout << "最小合計距離評価値: " << *minSumofDistance << endl;
	cout << "t: " << mint( 0 ) << ", " << mint( 1 ) << ", " << mint( 2 ) << endl;
	cout << "R ID: " << minRID << endl;
	cout << "R: " << minR( 0, 0 ) << "\t" << minR( 0, 1 ) << "\t" << minR( 0, 2 ) << endl;
	cout << "   " << minR( 1, 0 ) << "\t" << minR( 1, 1 ) << "\t" << minR( 1, 2 ) << endl;
	cout << "   " << minR( 2, 0 ) << "\t" << minR( 2, 1 ) << "\t" << minR( 2, 2 ) << endl;
	cout << "Degree: " << minDegree( 0 ) << ", " << minDegree( 1 ) << ", " << minDegree( 2 ) << endl;
	cout << "DistanceMeans: " << distanceMeans << endl << endl;


#ifdef ESRESULT

	/*** 全探索結果の出力 ***/
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr targetPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr modelPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	pcl::PointXYZRGB temp2;	// 点座標の一時格納用変数

	/* 探索対象点群 */
	for( int i = 0; i < cloudT->size(); i++ ){
			
		/* 探索対象点群を DF 座標系へ変換：DF 座標系は点群の重心が原点 */
		temp2.x = cloudT->points[ i ].x - ( gravX / (double)TOMETER );
		temp2.y = cloudT->points[ i ].y - ( gravY / (double)TOMETER );
		temp2.z = cloudT->points[ i ].z - ( gravZ / (double)TOMETER );
		temp2.r = 0;
		temp2.g = 255;
		temp2.b = 0;
		targetPointCloud->points.push_back( temp2 );
	}
	targetPointCloud->width = targetPointCloud->points.size();
	targetPointCloud->height = 1;

	/* モデル点群 */
	for( int i = 0; i < cloudM->size(); i++ ){
			
		/* モデル点群を DF 座標系へ変換：DF 座標系は点群の重心が原点 */
		temp2.x = cloudM->points[ i ].x - ( mGravX / (double)TOMETER );
		temp2.y = cloudM->points[ i ].y - ( mGravY / (double)TOMETER );
		temp2.z = cloudM->points[ i ].z - ( mGravZ / (double)TOMETER );
		temp2.r = 255;
		temp2.g = 0;
		temp2.b = 0;
		modelPointCloud->points.push_back( temp2 );
	}
	modelPointCloud->width = cloudM->points.size();
	modelPointCloud->height = 1;

	/* モデル点群の回転並行移動 */
	for( int i = 0; i < cloudM->size(); i++ ){
		
		p( 0 ) = modelPointCloud->points[ i ].x;
		p( 1 ) = modelPointCloud->points[ i ].y;
		p( 2 ) = modelPointCloud->points[ i ].z;
		
		rp = R[ minRID ] * p + mint;

		temp2.x = rp( 0 );
		temp2.y = rp( 1 );
		temp2.z = rp( 2 );
		temp2.r = modelPointCloud->points[ i ].r;
		temp2.g = modelPointCloud->points[ i ].g;
		temp2.b = modelPointCloud->points[ i ].b;
		transformedPointCloud->points.push_back( temp2 );
	}
	transformedPointCloud->width = modelPointCloud->points.size();
	transformedPointCloud->height = 1;

	/* 点群の統合 + 可視化 + 点群出力保存 */
	mergedPointDataViewerRGB( targetPointCloud, modelPointCloud, BeforeExhaustiveSearchPCDFileName, BeforeExhaustiveSearchPLYFileName );
	mergedPointDataViewerRGB( targetPointCloud, transformedPointCloud, AfterExhaustiveSearchPCDFileName, AfterExhaustiveSearchPLYFileName );

#endif

#ifdef ITERATIVECLOSESTPOINT
	
	/*** ICP を用いた2点群の最適化 ***/
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr newCloudM( new pcl::PointCloud< pcl::PointXYZRGB > );		// 位置合わせ後のモデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedIcpCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// ICP 後の統合点群
	
	/* ICP */
	ICPOptomizationRGB( transformedPointCloud, targetPointCloud, newCloudM, mergedIcpCloud );

#endif


	return 0;
}
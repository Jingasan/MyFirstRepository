/***** ディスタンスフィールドの作成 *****/


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** 探索対象点群に対してディスタンスフィールドを作成 ***/
void CreateDistanceFieldforPCDModel(
	int DistanceFieldDevideSize,				// DF の分割数( DF のボクセルの分解能 )
	int** DistanceField,						// DF
	int** ClosestPointID,						// 最近点の ID
	int* ModelPointCloudSize,					// 探索対象点群数
	struct ModelPointCloud** modelpointcloud,	// 探索対象点群( DF の座標系に直したもの, 単位:mm )
	struct ModelInformation* modelinformation,	// 探索対象点群の各種情報
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud	// 探索対象点群( 単位:m )
){
	
	/* 変数の定義 */
	int i, j, k, l;				// DF の ID が i, j, k
    int dist;					// ボクセルとパッチとの距離
    int *DFtemp;				// DF 作成のために一時的に保存する箱
    int *CPtemp;				// CP 作成のために一時的に保存する箱
    int ID;						// DF のインデックス
    int *stack;					// DF 作成に使用
    int top;					// stack の top を指す
    int infiniFlag;				// 例外処理のフラグ
    int tempi;					// 最適な i
    int flag;					// 例外処理するかどうか
    int size2;					// size2 = iSize * jSize;
    int size3;					// size3 = iSize * jSize * iSize;
    double *x;					// 下側包落線の交点を保存
    int iSize, jSize, kSize;	// DF のサイズ 
	double gravX, gravY, gravZ;	// 点群の中心座標をモデル座標系の原点とする
	
	/* 初期化 */
	modelinformation->Xmin = INFINITY;
	modelinformation->Ymin = INFINITY;
	modelinformation->Zmin = INFINITY;
	modelinformation->Xmax = -INFINITY;
	modelinformation->Ymax = -INFINITY;
	modelinformation->Zmax = -INFINITY;
	gravX = 0;
	gravY = 0;
	gravZ = 0;
	
	/* 点群の端点の算出 */
	for( size_t i = 0; i < cloud->points.size(); i++ ){
		
		/* X, Y, Z の最小値を設定 */
		if( cloud->points[ i ].x * TOMETER < modelinformation->Xmin ) modelinformation->Xmin = cloud->points[ i ].x * TOMETER;
		if( cloud->points[ i ].y * TOMETER < modelinformation->Ymin ) modelinformation->Ymin = cloud->points[ i ].y * TOMETER;
		if( cloud->points[ i ].z * TOMETER < modelinformation->Zmin ) modelinformation->Zmin = cloud->points[ i ].z * TOMETER;
		
		/* X, Y, Z の最大値を設定 */
		if( cloud->points[ i ].x * TOMETER > modelinformation->Xmax ) modelinformation->Xmax = cloud->points[ i ].x * TOMETER;
		if( cloud->points[ i ].y * TOMETER > modelinformation->Ymax ) modelinformation->Ymax = cloud->points[ i ].y * TOMETER;
		if( cloud->points[ i ].z * TOMETER > modelinformation->Zmax ) modelinformation->Zmax = cloud->points[ i ].z * TOMETER;
	}
	
	/* 点群の重心の算出 */ 
	gravX = ( modelinformation->Xmax + modelinformation->Xmin ) / 2;
	gravY = ( modelinformation->Ymax + modelinformation->Ymin ) / 2;
	gravZ = ( modelinformation->Zmax + modelinformation->Zmin ) / 2;
	
	/* 点群の重心をモデル情報として格納 */
	modelinformation->gravX = gravX;
	modelinformation->gravY = gravY;
	modelinformation->gravZ = gravZ;

	/* 点群の重心を原点に設定 */
	modelinformation->Xmin -= gravX; modelinformation->Xmax -= gravX;
	modelinformation->Ymin -= gravY; modelinformation->Ymax -= gravY;
	modelinformation->Zmin -= gravZ; modelinformation->Zmax -= gravZ;
	
	/* 点群の最大幅の算出 */
	modelinformation->deltaX = modelinformation->Xmax - modelinformation->Xmin;
	modelinformation->deltaY = modelinformation->Ymax - modelinformation->Ymin;
	modelinformation->deltaZ = modelinformation->Zmax - modelinformation->Zmin;
    
	/* 最も大きい値を持つ点群幅の算出 X方向 or Y方向 or Z方向 */
	if( modelinformation->deltaX > modelinformation->deltaY ){
		if( modelinformation->deltaX > modelinformation->deltaZ ){
			modelinformation->max = modelinformation->deltaX;
		}else{
			modelinformation->max = modelinformation->deltaZ;
		}
	}else{
		if( modelinformation->deltaY > modelinformation->deltaZ ){
			modelinformation->max = modelinformation->deltaY;
		}else{
			modelinformation->max = modelinformation->deltaZ;
		}
	}

	/* ディスタンスフィールドのサイズの算出 */
	modelinformation->iSize = (int)( ( modelinformation->deltaX / modelinformation->max ) * DistanceFieldDevideSize );
	modelinformation->jSize = (int)( ( modelinformation->deltaY / modelinformation->max ) * DistanceFieldDevideSize );
	modelinformation->kSize = (int)( ( modelinformation->deltaZ / modelinformation->max ) * DistanceFieldDevideSize );
	iSize = modelinformation->iSize;	// DF の i 方向のサイズ
	jSize = modelinformation->jSize;	// DF の j 方向のサイズ
	kSize = modelinformation->kSize;	// DF の k 方向のサイズ
	size2 = iSize * jSize;				// DF の面積
	size3 = iSize * jSize * kSize;		// DF の体積
    
	/* メモリの確保と初期化 */ 
	( *DistanceField ) = new int[ size3 ];	// DF のメモリ確保
	( *ClosestPointID ) = new int[ size3 ];	// 最近点のメモリ確保
	DFtemp = new int[ size3 ];
	CPtemp = new int[ size3 ];
	for( i = 0; i < size3; i++ ){
		DFtemp[ i ] = INFINITY;
		( *ClosestPointID )[ i ] = INFINITY;
	}
	stack = new int[ DistanceFieldDevideSize ];
	x = new double[ DistanceFieldDevideSize ];
    


	/*** ディスタンスフィールドの二値化 ***/
	cout << "--- 【 BinarizationDFforPCDModel関数 】DFを二値化 ---" << endl;
	BinarizationDFforPCDModel(
		DistanceFieldDevideSize,	// DF の分割数( DF のボクセルの分解能 )
		DFtemp,						// DF 作成のために一時的に保存する箱
		ClosestPointID,				// 最近点の ID
		modelpointcloud,			// 3次元点群
		ModelPointCloudSize,		// 3次元点群の数
		modelinformation,			// 3次元点群の各種情報
		cloud,						// 入力3次元点群
		gravX,						// 3次元点群の X 方向の重心
		gravY,						// 3次元点群の Y 方向の重心
		gravZ						// 3次元点群の Z 方向の重心
	);	// ここでボクセルとパッチとの距離が 0.5 以内のときに距離を 0 として DF に保存する



    /*** ディスタンスフィールドの作成 ***/

	/* Step1｜縦方向に距離を計算する */
    for( k = 0; k < kSize; k++ ){
		for( i = 0; i < iSize; i++ ){
			dist = INFINITY;
			ID = INFINITY;
			for( j = 0; j < jSize; j++ ){
				if( dist != INFINITY ){
					dist++;
				}
				/* 距離が0の点を見つけたら距離0を設定，その最近点IDを登録 */
				if( DFtemp[ k * size2 + j * iSize + i ] == 0 ){
					dist = 0;
					ID = ( *ClosestPointID )[ k * size2 + j * iSize + i ];
				}
				( *DistanceField )[ k * size2 + j * iSize + i ] = dist;
				( *ClosestPointID )[ k * size2 + j * iSize + i ] = ID;
			} // for j

			dist = INFINITY;
			ID = INFINITY;
			for( j = jSize - 1; j >= 0; j-- ){
				if( dist != INFINITY ){
					dist++;
				}
				if( DFtemp[ k * size2 + j * iSize + i ] == 0 ){
					dist = 0;
					ID = ( *ClosestPointID )[ k * size2 + j * iSize + i ];
				}
				if( dist < ( *DistanceField )[ k * size2 + j * iSize + i ] ){
					( *DistanceField )[ k * size2 + j * iSize + i ] = dist;
					( *ClosestPointID )[ k * size2 + j * iSize + i ] = ID;
				}
			} // for j
		} // for i
	} // for k
	
	/* Step2｜横方向に距離を計算する */
	for( k = 0; k < kSize; k++ ){
		for( j = 0; j < jSize; j++ ){
			top = 0;
			infiniFlag = 0;
			
			for( l = 0; l < iSize - 1; l++ ){
				if( ( *DistanceField )[ k * size2 + j * iSize + l ] != INFINITY && ( *DistanceField )[ k * size2 + j * iSize + l + 1 ] != INFINITY ){
					x[ top ] = (double)( 1 + 2 * l + ( ( *DistanceField )[ k * size2 + j * iSize + l + 1 ] * ( *DistanceField )[ k * size2 + j * iSize + l + 1 ] ) - ( ( *DistanceField ) [ k * size2 + j * iSize + l ] * ( *DistanceField )[ k * size2 + j * iSize + l ] ) ) / 2;
					stack[ top ] = l;
					top++;
					stack[ top ] = l + 1;
					break;
				} // if
				if( l == ( iSize - 2 ) ){
					infiniFlag = 1;
				}
			} // for l

			if( infiniFlag == 0 ){
				for( i = l + 2; i < iSize; i++ ){
					x[ top ] = (double)( ( stack[ top ] * stack[ top ]) - ( i * i ) + ( ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] * ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] ) - ( ( *DistanceField )[ k * size2 + j * iSize + i ] * ( *DistanceField )[ k * size2 + j * iSize + i ] ) ) / ( 2 * ( stack[ top ] - i ) );
					if( ( *DistanceField )[ k * size2 + j * iSize + i ] != INFINITY ){
						while( x[ top ] < x[ top - 1 ] && top > 0 ){
							top--;
							x[ top ] = (double)( ( stack[ top ] * stack[ top ] ) - ( i * i ) + ( ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] * ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] ) - ( ( *DistanceField )[ k * size2 + j * iSize + i ] * ( *DistanceField )[ k * size2 + j * iSize + i ] ) ) / ( 2 * ( stack[ top ] - i ) );
						}
						top++;
						stack[ top ] = i;
					}
				}//for i

				while( (double)( iSize - 1 ) < x[ top - 1 ] ){
					top--;
				}
				
				for( i = iSize - 1; i >= 0; i-- ){
					while( top > 0 && (double)i <= x[ top - 1 ] ){
						top--;
						if( top == 0 ) break;
					}
					DFtemp[ k * size2 + j * iSize + i ] = ( ( i - stack[ top ] ) * ( i - stack[ top ] ) ) + ( ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] * ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] );
					CPtemp[ k * size2 + j * iSize + i ] = ( *ClosestPointID )[ k * size2 + j * iSize + stack[ top ] ];
				}
			}else{
				flag = 0;
				for( i = 0; i < iSize; i++ ){
					if( ( *DistanceField )[ k * size2 + j * iSize + i ] != INFINITY ){
						flag = 1;
						tempi = i;
						
						for( l = 0; l < iSize; l++ ){
							dist = ( l - tempi ) * ( l - tempi ) + ( *DistanceField )[ k * size2 + j * iSize + tempi ] * ( *DistanceField )[ k * size2 + j * iSize + tempi ];
							if( DFtemp[ k * size2 + j * iSize + l ] >= dist ){
								DFtemp[ k * size2 + j * iSize + l ] = dist;
								CPtemp[ k * size2 + j * iSize + l ] = ( *ClosestPointID )[ k * size2 + j * iSize + tempi ];
							}
						} // for l
					}else{
						if( flag == 0 ){
							DFtemp[ k * size2 + j * iSize + i ] = INFINITY;
						} // if
					}
				}
			}
		}
	}

	/* Step3｜奥方向に距離を計算する */
	for( j = 0; j < jSize; j++ ){
		for( i = 0; i < iSize; i++ ){
			top = 0;
			infiniFlag = 0;
			for( l = 0; l < kSize - 1; l++ ){
				if( DFtemp[ l * size2 + j * iSize + i ] != INFINITY && DFtemp[ l * size2 + j * iSize + i + 1 ] != INFINITY ){
					x[ top ] = (double)( 1 + 2 * l + DFtemp[ ( l + 1 ) * size2 + j * iSize + i ] - DFtemp[ l * size2 + j * iSize + i ] ) / 2;
					stack[ top ] = l;
					top++;
					stack[ top ] = l + 1;
					break;
				}
				if( l == ( kSize - 2 ) ){
					infiniFlag = 1;
				} // if
			}

			if( infiniFlag == 0 ){
				for( k = l + 2; k < kSize; k++ ){
					x[ top ] = (double)( ( stack[ top ] * stack[ top ] ) - ( k * k ) + DFtemp[ stack[ top ] * size2 + j * iSize + i ] - DFtemp[ k * size2 + j * iSize + i ] ) / ( 2 * ( stack[ top ] - k ) );
					if( DFtemp[ k * size2 + j * iSize + i ] != INFINITY ){
						while( x[ top ] < x[ top - 1 ] && top > 0 ){
							top--;
							x[ top ] = (double)( ( stack[ top ] * stack[ top ] ) - ( k * k ) + DFtemp[ stack[ top ] * size2 + j * iSize + i ] - DFtemp[ k * size2 + j * iSize + i ] ) / ( 2 * ( stack[ top ] - k ) );
						}
						top++;
						stack[ top ] = k;
					}
				}

				while( (double)( kSize - 1 ) < x[ top - 1 ] ){
					top--;
				}

				for( k = kSize - 1; k >= 0; k-- ){
					while( top > 0 && (double)k <= x[ top - 1 ] ){
						top--;
						if( top == 0 ){
							break;
						}
					}
					( *DistanceField )[ k * size2 + j * iSize + i ] = ( ( k - stack[ top ] ) * ( k - stack[ top ] ) ) + DFtemp[ stack[ top ] * size2 + j * iSize + i ];
					( *ClosestPointID )[ k * size2 + j * iSize + i ] = CPtemp[ stack[ top ] * size2 + j * iSize + i ];
				}
			}else{
				flag = 0;
				for( k = kSize - 1; k >= 0; k-- ){
					if( ( *DistanceField )[ k * size2 + j * iSize + i ] != INFINITY ){
						flag = 1;
						tempi = k;
						for( l = 0; l < kSize; l++ ){
							dist = ( l - tempi ) * ( l - tempi ) + DFtemp[ tempi * size2 + j * iSize + i ] * DFtemp[ tempi * size2 + j * iSize + i ];
							if( ( *DistanceField )[ l * size2 + j * iSize + i ] >= dist ){
								( *DistanceField )[ l * size2 + j * iSize + i ] = dist;
								( *ClosestPointID )[ l * size2 + j * iSize + i ] = CPtemp[ tempi * size2 + j * iSize + i ];
							}
						}
					}else{
						if( flag == 0 ){
							( *DistanceField )[ k * size2 + j * iSize + i ] = INFINITY;
						}
					}
				} // for k
			}
		} // for i
	} // for j


	/*** ディスタンスフィールドと最近点の出力 ***/
	cout << "・・・ディスタンスフィールド・最近点を出力・・・" << endl;
	FILE *FPDF, *FPCP, *FPDFCP;
	FPDF = fopen( "Output/DistanceField/DistanceField.csv", "w" );
	FPCP = fopen( "Output/DistanceField/ClosestPoint.csv", "w" );
	FPDFCP = fopen("Output/DistanceField/DFandCP.csv", "w" );
	fprintf( FPDF, "ﾃﾞｰﾀ形式,3,\nmemo1,\nX,Y,Z,Distance\n" );
	fprintf( FPCP, "ﾃﾞｰﾀ形式,3,\nmemo1,\nX,Y,Z,ClosestPoint\n" );
	fprintf( FPDFCP, "ﾃﾞｰﾀ形式,3,\nmemo1,\nX,Y,Z,DistanceField,ClosestPoint\n" );
	
	/* ディスタンスフィールドと最近点の書き込み */
	for( k = 0; k < kSize; k++ ){
		for( j = 0; j < jSize; j++ ){
			for( i = 0; i < iSize; i++ ){
				fprintf( FPDF, "%d,%d,%d,%d,\n", i, j, k, ( *DistanceField )[ k * size2 + j * iSize + i ] );
				fprintf( FPCP, "%d,%d,%d,%d,\n", i, j, k, ( *ClosestPointID )[ k * size2 + j * iSize + i ] );
				fprintf( FPDFCP,"%d,%d,%d,%d,%d\n",
					i, j, k, ( *DistanceField )[ k * size2 + j * iSize + i ], ( *ClosestPointID )[ k * size2 + j * iSize + i ] );
			} // for i
		} // for j
	} // for k
	
	fclose( FPDF );
	fclose( FPCP );
	fclose( FPDFCP );
	


	/* モデル点群を出力する */
	cout << "・・・モデルの点群を出力・・・" << endl;
	FILE *FPMODELPOINTCLOUD;
	FPMODELPOINTCLOUD = fopen( "Output/DistanceField/PCDModelPointCloud.csv", "w" );
	fprintf( FPMODELPOINTCLOUD, "ﾃﾞｰﾀ形式,2,\nmemo1,\nX,Y,Z,\n" );
	for( i = 0; i < *ModelPointCloudSize; i++ ){
		fprintf( FPMODELPOINTCLOUD, "%f,%f,%f,\n", ( *modelpointcloud )[ i ].Xmodel, ( *modelpointcloud )[ i ].Ymodel, ( *modelpointcloud )[ i ].Zmodel );
	}
	fclose( FPMODELPOINTCLOUD );

#ifdef COLOREVALUATE
 	
	/* 最近点ID付きRGB情報を出力 */
	cout << "・・・ディスタンスフィールドの色を出力・・・" << endl;
	FILE *FPCPRGB;
	FPCPRGB = fopen( "Output/CPRGBdata.txt", "w" );
	fprintf( FPCPRGB, "ClosestPointID:R,G,B\n" );
	for( i = 0; i < *ModelPointCloudSize; i++ ){
		fprintf( FPCPRGB, "%d:%d,%d,%d\n", i, ( *DistanceFieldRGB )[ i ].R, ( *DistanceFieldRGB )[ i ].G, ( *DistanceFieldRGB )[ i ].B );
	}
	fclose( FPCPRGB );

#endif // COLOREVALUATE

	delete []CPtemp;
	delete []DFtemp;
	delete []stack;
	delete []x;
}



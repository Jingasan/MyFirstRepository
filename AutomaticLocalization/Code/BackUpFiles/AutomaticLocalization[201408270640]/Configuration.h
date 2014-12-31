/***** 設定ヘッダファイル *****/
// PCL 警告非表示コード：4996;4819;4503;4305;4244;4005;


/*** フラグの定義 ***/
#define COMMENT		// コマンドライン出力 ON:出力 OFF:非出力
#define VISUALIZE	// 点群の可視化


/*** 定数の定義 ***/

/* createDistanceField 関数 */
#define INFINITY 99999			// DF 作成時の初期化
#define TOMETER 1000			// 3次元点群座標の単位変換： m ⇒ mm
#define DISTANCEFIELDSIZE 100	// DF の分解能の既定値

/* 全探索 */
#define LIMIT_X 2			// 並進 X の探索範囲の絞り込み：上限値 / LIMIT_X, 下限値 / LIMIT_X
#define LIMIT_Y 2			// 並進 Y の探索範囲の絞り込み：上限値 / LIMIT_Y, 下限値 / LIMIT_X
#define LIMIT_Z 2			// 並進 Z の探索範囲の絞り込み：上限値 / LIMIT_Z, 下限値 / LIMIT_X
#define SAMP_R_LEVEL 1		// 回転空間の均等なサンプリングの解像度レベル： 0, 1, 2, 3
#define SAMP_X 10			// 並進 X のサンプリング幅
#define SAMP_Y 10			// 並進 Y のサンプリング幅
#define SAMP_Z 10			// 並進 Z のサンプリング幅
#define SAMP_THETA 10		//
#define SAMP_PHI 10			//
#define SAMP_PSI 10			//
#define DISTANCE_ERROR 10	// モデル点群と探索対象点群の1点当たりの DF 評価値の閾値：評価値がこの値よりも小さくなる R と t を候補として取得

/* サンプリングされた回転行列の基となる四元数と回転角θΦΨ */
const char QuaternionFileName1[] = "Output/SamplingR/72quaternion.csv";
const char QuaternionFileName2[] = "Output/SamplingR/576quaternion.csv";
const char QuaternionFileName3[] = "Output/SamplingR/4608quaternion.csv";
const char QuaternionFileName4[] = "Output/SamplingR/36864quaternion.csv";
const char DegreeFileName1[] = "Output/SamplingR/72degree.csv";
const char DegreeFileName2[] = "Output/SamplingR/576degree.csv";
const char DegreeFileName3[] = "Output/SamplingR/4608degree.csv";
const char DegreeFileName4[] = "Output/SamplingR/36864degree.csv";

/* 出力ファイル名 */
const char BeforeExhaustiveSearchPCDFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudDataBefore.pcd";
const char BeforeExhaustiveSearchPLYFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudDataBefore.ply";
const char AfterExhaustiveSearchPCDFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudData.pcd";
const char AfterExhaustiveSearchPLYFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudData.ply";


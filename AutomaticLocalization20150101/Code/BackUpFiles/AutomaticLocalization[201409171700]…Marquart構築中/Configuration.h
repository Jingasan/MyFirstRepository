/***** 設定ヘッダファイル *****/
// PCL 警告非表示コード：4996;4819;4503;4521;4305;4267;4244;4005;


/*** フラグの定義 ***/

/* 処理の根幹となるフラグ */
#define CREATEDISTANCEFIELD		// ON: DF 作成実行
#define EXHAUSTIVESEARCH		// ON: 全探索実行
#define MARQUARDT				// ON: マーカート法による最適化実行

/* その他処理用フラグ */
#define COMMENT					// コマンドライン出力 ON:出力 OFF:非出力
#define VISUALIZE				// 点群の可視化
#define ESRESULT				// 全探索結果の出力
#define ITERATIVECLOSESTPOINT	// ICP による最適化 ON: ICP 実行 OFF: ICP 非実行 
#define ICPRESULT				// ICP 結果の出力
#define OUTPUTEXECUTIONTIME		// 実行時間の出力


/*** 定数の定義 ***/

/* ディスタンスフィールド関連 */
#define INFINITY 99999			// DF 作成時の初期化
#define TOMETER 1000			// 3次元点群座標の単位変換： m ⇒ mm
#define DISTANCEFIELDSIZE 100	// DF の分解能の既定値

/* 全探索関連 */
#define LIMIT_X 2			// 並進 X の探索範囲の絞り込み：上限値 / LIMIT_X, 下限値 / LIMIT_X
#define LIMIT_Y 2			// 並進 Y の探索範囲の絞り込み：上限値 / LIMIT_Y, 下限値 / LIMIT_X
#define LIMIT_Z 2			// 並進 Z の探索範囲の絞り込み：上限値 / LIMIT_Z, 下限値 / LIMIT_X
#define SAMP_R_LEVEL 1		// 回転空間の均等なサンプリングの解像度レベル： 0, 1, 2, 3
#define SAMP_X 10			// 並進 X のサンプリング幅
#define SAMP_Y 10			// 並進 Y のサンプリング幅
#define SAMP_Z 10			// 並進 Z のサンプリング幅

/* ICP 関連 */
#define ICP_LOOP_MAX 50							// ICPの最大反復回数
#define RANSAC_OUTLIER_REJECTION_THRESHOLD 5	// RANSACを用いた外れ値除去の閾値設定
#define MAX_CORRESPONDENCE_DISTANCE 10			// 対応点間距離の最大値


/*** 入力ファイル群 ***/

/* サンプリングされた回転行列の基となる四元数と回転角θΦΨ */
const char QuaternionFileName1[] = "Output/SamplingR/72quaternion.csv";
const char QuaternionFileName2[] = "Output/SamplingR/576quaternion.csv";
const char QuaternionFileName3[] = "Output/SamplingR/4608quaternion.csv";
const char QuaternionFileName4[] = "Output/SamplingR/36864quaternion.csv";
const char DegreeFileName1[] = "Output/SamplingR/72degree.csv";
const char DegreeFileName2[] = "Output/SamplingR/576degree.csv";
const char DegreeFileName3[] = "Output/SamplingR/4608degree.csv";
const char DegreeFileName4[] = "Output/SamplingR/36864degree.csv";


/*** 出力ファイル群 ***/

/* 全探索の前後結果 */
const char BeforeExhaustiveSearchPCDFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudDataBefore.pcd";	// 全探索前の2点群の統合結果 
const char BeforeExhaustiveSearchPLYFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudDataBefore.ply";	// 全探索前の2点群の統合結果
const char AfterExhaustiveSearchPCDFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudDataAfter.pcd";		// 全探索後の2点群の統合結果
const char AfterExhaustiveSearchPLYFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudDataAfter.ply";		// 全探索後の2点群の統合結果

/* ICP アルゴリズム後の結果 */
const char IcpRtFileName[] = "Output/ICPResult/ICP_Rt.csv";
const char IcpRMSEFileName[] = "Output/ICPResult/ICP_RMSE.csv";
const char IcpPCDFileName[] = "Output/ICPResult/MergedPointCloudDataAfterICP.pcd";	// ICP を用いた最適化後の2点群の統合結果
const char IcpPLYFileName[] = "Output/ICPResult/MergedPointCloudDataAfterICP.ply";	// ICP を用いた最適化後の2点群の統合結果

/* 実行時間の出力ファイル */
const char ExecutionTimeFileName[] = "Output/ExecutionTime/ExecutionTime.csv";
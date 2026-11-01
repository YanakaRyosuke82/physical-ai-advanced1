## 初回の起動

1. 今回のデモでは推論のためOpen APIを使用する。
OpenAI APIキーを使用する。

```bash
read -s -p "OpenAI API key: " OPENAI_API_KEY
export OPENAI_API_KEY
echo
```

2. 最初はAPI・移動なしで、情報取得と記録を確認します。

```bash
bash run.sh --inspect-only
```

3. `scenarios/instruction.txt` の日本語指示と `scenarios/targets.json` の座標を編集します。
. 計画だけを試す場合：

```bash
bash run.sh --plan-only
```

4. 計画・検証後に実行する場合：

```bash
bash run.sh
```

AIの計画と数値が表示されます。最後の確認で `y` と入力すると実行します。Enterだけなら動かさず終了します。`--plan-only`の結果の再生ではなく、その実行時の現在状態から改めて計画します。入力待ちの間も状態・rosbag記録は継続します。

Gazebo/RVizによる他の操作やPlanning Sceneの編集は、この実行中に行わないでください。Ctrl+Cは進行中Actionのキャンセルを試み、ログとrosbagを終了します。

## フォルダ構成

- `main.py`：起動、入力、実行前の確認、全体の進行。
- `ai/planner.py`：APIのfunction calling、ツール選択、失敗結果からの修正。
- `ai/schema.py`：動作仕様の型と範囲検査。
- `robot/model.py`：実環境のURDF/SRDF読込、FK、対応するCRANE形状の解析IK、制限の整理。
- `robot/bridge.py`：ROS通信、状態購読、サービス、MoveIt計画・実行。
- `robot/trajectory.py`：連続補間、揺動波形、速度・加速度・補間区間の検査。
- `robot/validation.py`：AI提出候補の必須検証。合格した候補のみ実行対象にする。
- `recording/logger.py`：CSVとrosbag2の記録・終了処理。
- `config/settings.json`：モデル、検査上限、APIループ上限、記録対象。
- `scenarios/instruction.txt`：ユーザーが編集する日本語指示。
- `scenarios/targets.json`：固定A-E座標。以前のtargets.jsonと置換可能。
- `runs/`：実行ごとの入力、モデル、計画、CSV、rosbag、結果。
- `tests/`：ROS/APIなしで実行できるテスト。fixturesはテスト用で、実行時のモデルに使わない。

## AIの役割

固定A-Eに対し、AIが各点と辺中点の姿勢を計画します。AIはロボット情報・現在状態を取得し、必要に応じてIKツールで数値を求め、関節姿勢の候補を提出します。FK・衝突・軌道検証の実結果を受け取り、候補を修正して再提出します。検証成功のplan_idがなければ実行できません。APIの失敗時に固定動作へ自動フォールバックする処理はありません。

利用ツール：`get_robot_context`、`get_current_state`、`solve_target_poses`、`evaluate_pose`、`validate_plan`、`finish_plan`。

幾何計算・補間・衝突判定そのものはPythonとMoveItで行います。AIは計画・姿勢選択・検証ツールの呼び出し・修正を担当します。AIに検証結果を推測させません。数学的な計算支援としてIKツールを使用します。説明文が成功と述べても、検証に合格しない候補は採用しません。

4関節の機構なので、先端位置を固定したときのjoint1-3は強く制約されます。手首姿勢は自由に選べる範囲があります。現在の補間backendはjoint3負側の解を使い、未対応の構造やIK枝は拒否します。汎用ロボット対応や、指先接触点で壁へ描画するモデルではありません。SRDFのendtipは手首回転原点と一致します。

AIの中間関節姿勢は座標誤差1mm・姿勢整合性を検査します。実軌道は目標座標を維持した解析IK補間で作るため、提出値をそのまま丸めて再生するのではありません。AIが選んだ各点・中間点の傾きと動作条件を使います。

指定の達成には数値検証と、人による指示内容の確認を併用します。日本語の意味と数値の対応を完全に形式検証するものではありません。曖昧な指示・実現不能な条件では、AIは理由や代案を提示し、実行しない仕様です。

## ログ

全CSVはrun_id、phase、sim_s、wall_time_nsを共通列として持ちます。欠損はPython内でNone、CSVでは文字列 `None`。実測0とは区別します。JSONの欠損はJSON標準のnullです。ログ解析でCSVを読み込む場合は `None` を欠損値へ変換してください。

- `joints.csv`：/joint_statesで配信された全関節の位置・速度・effort、メッセージ時刻。速度・effortが未配信ならNone。
- `links.csv`：10Hz程度でURDFから求めた各リンクの位置・Quaternion。欠けた関節をゼロ補完せず、その先のリンク計算を省略。
- `controller_state.csv`：取得できるコントローラの目標・実測・誤差（位置・速度・加速度）。未提供はNone。
- `controller_messages.csv`：コントローラの元メッセージのJSON表現。
- `simulator_state.csv`：Gazebo LinkStates/ModelStatesが配信されている場合のみ、そのメッセージ。配信を新たに有効化するプラグイン変更は行わない。
- `commands.csv`：実行へ送る時刻付き関節指令。
- `ai_history.csv`：AIのツール呼出し、候補姿勢、実際の検証結果。トークン数・API応答時間は保存しない。
- `validation.csv`：候補ごとの入力・検証結果。
- `events.csv`：計画採用・実行開始・完了・エラーなど。
- `rosbag/`：選択トピックの元ROSメッセージ。/tfと/tf_staticも対象。配信されたもののみ収録。
- `topics.json`：起動時に発見したトピックと型。
- `robot_description.urdf/.srdf`、`parameters.json`：実環境で取得したモデル・制限。
- `planning_scene.json`：起動時のMoveItシーン。
- `candidate_*.json`：採用候補の関節姿勢・軌道・開始状態。
- `result.json`：実行結果と最終位置誤差。

コントローラやGazebo専用トピックがなければ、そのCSVは生成されません。topic_inventoryとtopics.jsonから有無を確認できます。受信時刻とメッセージ時刻を区別し、CSVとrosbagを照合します。実行のイベント時刻は制御周期に同期した厳密な軌道開始時刻ではありません。

## 検証・監視の範囲

- URDFの関節制限と、取得できたMoveIt joint_limitsを使用。速度はconfig上限と取得値の小さい方。
- 加速度情報が取得できない場合はconfigの4rad/s²を工学的な検査上限として使用し、robot_context.jsonへその事実を記録。実機定格を保証する値ではない。
- 離散姿勢と補間中点のMoveIt衝突検査。連続経路全体の無衝突保証ではない。障害物はMoveItのPlanning Sceneに登録されたものが対象で、Gazebo表示のみの物体を自動認識しない。
- 実行前に状態と描画軌道を再検査。開始状態が変われば停止。
- 実行中のJointState途絶（2秒）を監視。コントローラ誤差が配信される場合、0.3rad超が0.5秒続くとキャンセルを試行。
- 最終関節誤差0.03radと先端位置誤差を確認。全通過点の実測合否は別途CSV解析で確認する。
- 実機向けの認証済み安全制御ではなく、講義のシミュレータ用デモ。

## 再現性

同じ指示でもAPIの結果は毎回同一とは限りません。実行時の指示・A-E・URDF/SRDF・制限・採用関節姿勢・軌道・検証結果・rosbagを実行フォルダに保存します。現版は保存済み計画の別時刻再生には対応せず、各起動で現在状態から再計画します。

## テストと制限

```bash
cd ~/ai_motion_demo
python3 -m unittest discover -v
```

構文、FK/IK、Noneと0の分離、動力学検査、ツールの境界をローカルで確認済み。ROS・Gazebo・実APIを組み合わせた本フォルダ版の統合試験はユーザー環境で実施する必要があります。API呼出し数は最大24ターン、候補検証は最大3回。設定ファイルで変更できます。APIキーは保存・同梱しません。

参考：OpenAI function calling https://developers.openai.com/api/docs/guides/function-calling

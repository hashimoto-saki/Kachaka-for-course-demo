#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import kachaka_api
import time
import openai
import sys
import speech_recognition as sr

client = kachaka_api.KachakaApiClient("10.40.20.201:26400")
# client = kachaka_api.KachakaApiClient()

def task_planning(user_question):
    # APIキーをファイルから読み込む
    API_KEY_FILE = "API_key.txt"

    try:
        with open(API_KEY_FILE, "r") as file:
            openai.api_key = file.read().strip()  # .strip() で余分な空白や改行を削除
    except Exception as e:
        sys.exit(f"APIキーの読み込みに失敗しました: {e}")

    # task_planning.txtの内容を読み込む
    try:
        with open("task_planning.txt", "r", encoding="utf-8") as f:
            task_planning_prompt = f.read()
    except Exception as e:
        sys.exit(f"タスクファイルの読み込みに失敗しました: {e}")

    full_Coalition = task_planning_prompt.replace("USER_INSTRACTION", user_question)

    # OpenAI APIに送信するメッセージを作成
    messages = [{'role': 'user', 'content': full_Coalition}]

    # OpenAI APIを呼び出してレスポンスを取得
    try:
        response = openai.ChatCompletion.create(
            model='gpt-4',
            messages=messages,
            temperature=0.0,
        )

    except openai.error.OpenAIError as e:
        sys.exit(f"OpenAI APIエラー: {e}")
    except Exception as e:
        sys.exit(f"エラーが発生しました: {e}")

    Task_Planning_output = ('\n' + response['choices'][0]['message']['content'] + '\n')
    data = Task_Planning_output.split('= ')[1]  # スペースを含めて分割

    # 括弧を除去し、カンマで分割
    data = data.strip('()')
    parts = data.split(',')

    # taskとlocationに適切に代入
    task = parts[0].strip()
    location = parts[1].strip(' )\n')

    # 結果の出力
    print(f"task = '{task}'")
    print(f"location = '{location}'")

    return task, location



def listen_from_microphone():
    # 音声認識オブジェクトの初期化
    recognizer = sr.Recognizer()
    
    # マイクロフォンの設定
    with sr.Microphone() as source:
        # ノイズの調整
        recognizer.adjust_for_ambient_noise(source)

        
        while True:
            # ユーザーの話す音声を待ち受ける
            print("マイクが起動しました。何か話してください。")
            client.speak("マイクが起動しました。何か話してください。")
            audio = recognizer.listen(source)
            try:
                # Googleの音声認識サービスを使用して音声をテキストに変換
                text = recognizer.recognize_google(audio, language="ja-JP")
                print("認識したテキスト：", text)
                return text
            except sr.UnknownValueError:
                # Google Speech Recognitionが音声を認識できなかった場合
                print("音声を認識できませんでした。もう一度話してください。")
                client.speak("音声を認識できませんでした。もう一度話してください。")
            except sr.RequestError as e:
                # Google Speech Recognitionへのリクエストでエラーが発生した場合
                print(f"Google Speech Recognitionサービスエラー; {e}")




# 実際にマイクから音声を認識してみる
user_question = listen_from_microphone()
# user_question = "please come to Home"
task, location = task_planning(user_question)

if task == 'move':
    client.speak("移動します")
    client.move_to_location(location)

elif task == 'bring':
    client.speak("運びます")
    client.move_shelf("S02", location)
    time.sleep(5)
    client.return_shelf()

elif task == 'return':
    client.speak("帰ります")
    client.return_home()
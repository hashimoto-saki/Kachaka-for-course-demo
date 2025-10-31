import speech_recognition as sr

#google text to speechを使う音声認識

def listen_from_microphone():
    # 音声認識オブジェクトの初期化
    recognizer = sr.Recognizer()
    
    # マイクロフォンの設定
    with sr.Microphone() as source:
        print("マイクが起動しました。何か話してください。")
        
        # ノイズの調整
        recognizer.adjust_for_ambient_noise(source)
        
        # ユーザーの話す音声を待ち受ける
        audio = recognizer.listen(source)
        
        try:
            # Googleの音声認識サービスを使用して音声をテキストに変換
            text = recognizer.recognize_google(audio, language="ja-JP")
            print("認識したテキスト：", text)
        except sr.UnknownValueError:
            # Google Speech Recognitionが音声を認識できなかった場合
            print("Google Speech Recognitionが音声を理解できませんでした。")
        except sr.RequestError as e:
            # Google Speech Recognitionへのリクエストでエラーが発生した場合
            print(f"Google Speech Recognitionサービスエラー; {e}")

# 実際にマイクから音声を認識してみる
listen_from_microphone()

#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import sounddevice as sd
import numpy as np
from gtts import gTTS
from pydub import AudioSegment
import tempfile
import os

def sound_synthesis_gtts(text, lang='en'):
    # gTTSオブジェクトを作成
    speech = gTTS(text=text, lang=lang, slow=False)
    
    # 一時ファイルにMP3形式で保存
    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".mp3")
    speech.save(temp_file.name)
    
    # pydubを使用してMP3ファイルを読み込み
    audio = AudioSegment.from_mp3(temp_file.name)
    
    # デバッグ情報
    print(f"Original sample rate: {audio.frame_rate}")
    print(f"Original channels: {audio.channels}")
    
    # サンプルレートを16000 Hzに設定
    device_sample_rate = 16000
    audio = audio.set_frame_rate(device_sample_rate)
    
    # チャンネル数を1に設定
    audio = audio.set_channels(1)
    
    # numpy配列に変換
    samples = np.array(audio.get_array_of_samples(), dtype=np.int16)
    
    # デバッグ情報
    print(f"Converted sample rate: {audio.frame_rate}")
    print(f"Converted channels: {audio.channels}")
    print(f"Samples shape: {samples.shape}")
    
    # チャンネル数に基づいて再生するためのデータを修正
    if audio.channels == 2:
        samples = samples.reshape((-1, 2))
    else:
        samples = samples.reshape((-1, 1))
    
    # オーディオデバイスを指定して再生
    sd.default.device = 'hw:2,0'  # Jabra Speak 710を使用 
    
    # デバッグ情報
    print(f"Using device: {sd.query_devices(sd.default.device)}")
    
    # 再生
    try:
        sd.play(samples, samplerate=device_sample_rate)
        sd.wait()  # 再生が完了するまで待機
    except Exception as e:
        print(f"Error during playback: {e}")
    
    # 一時ファイルを削除
    os.remove(temp_file.name)

if __name__ == "__main__":
    while True:
        input_prompt = input("テキストを入力してください（終了するには'q'を入力）: ")
        
        if input_prompt.lower() == 'q':
            break
        
        lang = input("言語を選択してください（'en' または 'ja'): ")
        if lang not in ['en', 'ja']:
            print("無効な言語です。デフォルトの'en'を使用します。")
            lang = 'en'
        
        # テキストを音声合成して再生
        sound_synthesis_gtts(input_prompt, lang)

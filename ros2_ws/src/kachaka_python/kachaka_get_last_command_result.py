#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import kachaka_api

# KACHAKA_API.txt ファイルからサーバーのアドレスを読み込む
with open('KACHAKA_API.txt', 'r') as file:
    KACHAKA_API = file.read().strip()  # ファイルから読み取り、余分な空白や改行を削除

client = kachaka_api.KachakaApiClient(KACHAKA_API)

client.move_to_location('L01')

# client.get_last_command_result() からタプルを取得
result_tuple = client.get_last_command_result()

# Result オブジェクトの取得と処理
result_obj = result_tuple[0]
if result_obj.success:
    print("Operation was successful.")
    # 成功した場合の追加の処理を行う
else:
    print("Operation failed.")

# Command オブジェクトの取得と処理
command_obj = result_tuple[1]
if hasattr(command_obj, 'move_to_location_command'):
    location_id = command_obj.move_to_location_command.target_location_id
    print(f"Moving to location ID: {location_id}")

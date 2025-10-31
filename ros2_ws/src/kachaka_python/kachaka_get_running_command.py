#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import kachaka_api

# KACHAKA_API.txt ファイルからサーバーのアドレスを読み込む
with open('KACHAKA_API.txt', 'r') as file:
    KACHAKA_API = file.read().strip()  # ファイルから読み取り、余分な空白や改行を削除

client = kachaka_api.KachakaApiClient(KACHAKA_API)

# pose = (0.0, 2.0, 0.0)
# print(pose)
# client.move_to_pose(*pose)
client.move_to_location('L01')
print(client.get_running_command())
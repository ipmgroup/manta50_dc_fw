import subprocess

# Получаем последний коммит GIT хеш
git_hash = subprocess.check_output(["git", "rev-parse", "HEAD"]).strip().decode('utf-8')

# Берем последние 4 байта хеша
last_four_bytes = git_hash[-8:]

# Создаем имя файла заголовка
header_file_name = "version.h"

# Формируем содержимое файла заголовка
header_content = f"""#ifndef VERSION_H
#define VERSION_H

#define GIT_HASH 0x{last_four_bytes}

#endif // VERSION_H
"""

# Записываем содержимое в файл
with open(header_file_name, "w") as header_file:
    header_file.write(header_content)

print(f"Файл {header_file_name} успешно создан с GIT_HASH: {last_four_bytes}")
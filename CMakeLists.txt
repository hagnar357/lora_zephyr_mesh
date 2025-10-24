cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meu_lora)

# Inclui o diretório de cabeçalhos
target_include_directories(app PRIVATE
    include
)

# Adiciona os arquivos fonte do projeto
target_sources(app PRIVATE
    src/main.c
    src/lora_mesh.c
)

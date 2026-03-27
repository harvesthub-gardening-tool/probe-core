# Sonde Core - Setup & Commandes

Ce document retrace la configuration de l'environnement de développement Rust pour l'ESP32-S3 sous Windows.

## 1. Prérequis Système (Windows)
Avant d'installer Rust, il faut les outils de compilation C++ de Microsoft :
1. Télécharger et installer les **Build Tools pour Visual Studio**.
2. Cocher la charge de travail **Développement Desktop en C++**.

## 2. Installation de Rust et des Outils Espressif
Ouvrir un terminal PowerShell et exécuter les commandes suivantes dans l'ordre :

```bash
# 1. Installer Rust (si ce n'est pas déjà fait via rustup-init.exe)
rustc --version

# 2. Installer l'outil d'environnement ESP
cargo install espup

# 3. Télécharger la toolchain Xtensa pour ESP32
espup install

# 4. Installer les outils de flashage USB
cargo install cargo-espflash
cargo install espflash
from setuptools import setup, find_packages
from pathlib import Path

package_name = 'taurasim'

def get_files(directory, pattern="**/*"):
    """
    Retorna uma lista de tuplas com os arquivos de um diretório usando pathlib.

    Args:
        directory (str): Caminho base do diretório.
        pattern (str): Padrão de correspondência para os arquivos.

    Returns:
        list: Lista de tuplas no formato esperado por data_files.
    """
    base_path = Path(directory)
    files = [str(p) for p in base_path.glob(pattern) if p.is_file()]
    return files

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Usa find_packages para descobrir pacotes automaticamente
    data_files=[
        ('share/' + package_name + '/config', get_files('config')),
        ('share/' + package_name + '/urdf', get_files('urdf')),
        ('share/' + package_name + '/meshes', get_files('meshes')),
        ('share/' + package_name + '/media', get_files('media')),
        ('share/' + package_name + '/sounds', get_files('sounds')),
        ('share/' + package_name + '/models', get_files('models')),
        ('share/' + package_name + '/worlds', get_files('worlds')),
        ('share/' + package_name + '/launch', get_files('launch')),
        ('share/' + package_name + '/scripts', get_files('scripts'))
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'vision_proxy = taurasim.scripts.vision_proxy:main',
            'keyboard_node = taurasim.scripts.keyboard_node:main',
            'keyboard_node2 = taurasim.scripts.keyboard_node2:main'
        ],
    },
    zip_safe=True,
)

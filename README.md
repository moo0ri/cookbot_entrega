# Cookbot-Robotic-Control

Projeto de controle e simulação de manipulador robótico (Cookbot) – Engenharia de Controle de Sistemas Robóticos

## Visão Geral

Este repositório contém:
- Modelagem e parâmetros do robô.
- Geração de trajetórias e análise de singularidade.
- Projeto, implementação e simulação de controlador PID.
- Gráficos de validação e análise comparativa teoria x simulação.
- Relatório detalhado e apresentação em vídeo.

## Como rodar

1. Instale as dependências:
pip install -r requirements.txt
*(Python 3.9.21, roboticstoolbox-python 1.1.1, numpy 1.26.4, sympy 1.13.3, matplotlib 3.5.1)*

2. Scripts principais estão na pasta `src/`:
- `Trajetória com controle.py` — simula o controle PID.
- `plot_traj_real.py` — plota a trajetória real simulada.
- `plot_traj_des.py` — plota trajetórias desejadas (teoria).
- `plot_torques.py` — plota os torques das juntas.

3. Dados de simulação ficam em `data/`.
4. Gráficos gerados ficam em `figuras/`.

## Estrutura sugerida do relatório (relatorio/RelatorioFinal.pdf)

1. **Definição da trajetória e análise de singularidade**: 
 - Escolha e justificativa da trajetória.
 - Análise matemática das singularidades ao longo da trajetória.
2. **Projeto do controlador linear (PID)**:
 - Dedução da lei de controle.
 - Cálculo e justificativa dos ganhos.
 - Gráficos comparativos: posição desejada x real, erro, sinais de controle.
3. **Resultados e Conclusão**:
 - Discussão dos resultados.
 - Pontos de melhoria e limitações do modelo.
4. **Apresentação (video/)**:
 - Link do vídeo explicativo (até 15 min).

## Para gerar os gráficos obrigatórios

- **Seguimento de trajetória**: Compare os plots de posição real vs. desejada (scripts `plot_traj_real.py`, `plot_traj_des.py`).
- **Erro de posição**: Gerado automaticamente nos scripts de simulação.
- **Sinais de controle (torques)**: Execute `plot_torques.py`.

## Créditos
- Autor: Vinícius Mori Sartor
- Disciplina
- Professores
- Universidade de São Paulo

---

**Observação**: Todo código, gráficos e relatório estão disponíveis neste repositório, conforme exigido na avaliação.

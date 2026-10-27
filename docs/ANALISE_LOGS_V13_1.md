# Ensacadeira 2 — revisão v13.1 após os logs de 26/10

Esta revisão incorpora os arquivos `putty 26-10.txt` e
`putty 26-10 sacos 50kg.txt`, além da orientação do usuário: quando o indicador
retornar `99999` ao chegar a 50 kg, contar esse ensaque como **50 kg**.

O sketch completo está em
`firmware/Ensacadeira2_RS485_SP3_Estavel_v13_1/Ensacadeira2_RS485_SP3_Estavel_v13_1.ino`.
A v12 original e a v13 entregue anteriormente permanecem inalteradas. Não foi
feita gravação na placa nem acesso ao Firebase de produção.

## O que os logs demonstram

Os dois arquivos identificam o firmware em operação como **v12**. Os horários
abaixo são os horários impressos nos próprios logs, sem conversão de fuso.

| Arquivo | Pacotes PKT completos | Trecho observável |
|---|---:|---|
| 25 kg | 154 | 39,498 segundos de pacotes |
| 50 kg | 753 | janela de aproximadamente 223 segundos, com interrupção do monitor |

No log de 25 kg, há **7 registros v12 para 5 descargas observadas**. Os pares
129/130 e 134/135 são duplicações: SP3 desligou e religou enquanto havia peso
alto, antes de o mesmo saco descarregar.

No log de 50 kg, os ciclos **203 e 204**, aproximadamente entre 15:19:12 e
15:19:16, também ocorreram antes de uma única descarga. SP3 voltou a ligar
quando o peso ainda estava em 494–499, sem duas leituras baixas anteriores.

Existem 33 resumos de ciclo v12 com peso `99999` no arquivo de 50 kg. Nos PKT
disponíveis, as 49 amostras `99999` coincidem com STS1 `C1` ou `D1`, ambos com
bit `0x40` ligado. A regra desta revisão se baseia na interpretação operacional
fornecida pelo usuário; não atribui uma medida precisa a esse código.

Embora o indicador tenha sido descrito como limitado a 499, os logs também
contêm valores numéricos como 500, 501, 503, 510 e 523. Por isso, `00499` não é
arredondado para `00500`: a conversão especial é acionada por `99999` dentro
do ciclo real. Quando acionada, vale a regra de 50 kg estimados para esse ciclo,
mesmo que também existam algumas leituras numéricas próximas de 50 kg.

O cabeçalho do arquivo de 50 kg ainda mostra **598 — MILHO QUEBRADO 25KG**.
O firmware preserva a seleção manual do produto; não troca o produto a partir
da saturação. É necessário conferir a seleção/lista na máquina para não
classificar ensaques de 50 kg como produto de 25 kg. A conversão de `99999` é
independente desse rótulo, pois descreve o limite físico informado do indicador.

## Regra de saturação incorporada

Durante um ciclo de produção:

1. `99999` é normalizado internamente para 500, mantendo SP3 e a detecção de
   descarga. O pacote inteiro não é descartado: há sacos em que todo o trecho
   SP3 ligado contém somente esse código.
2. A primeira ocorrência é sinalizada no checkpoint LittleFS. O sinalizador
   sobrevive a reinicialização, inclusive antes de formar um trio de leituras.
3. O registro final recebe `peso="00500"`, equivalente a **50,0 kg** com uma
   casa decimal; `estimado=true`, `confiavel=false`, `estavel=false`, qualidade
   `I` e critério `indicador_saturado_50kg_estimado`.
4. O diagnóstico mantém `peso_maximo=99999`. Se SP3 iniciou já com esse código,
   `peso_quando_sp3_ligou` também permanece 99999. O monitor PKT continua
   mostrando os bytes/pesos recebidos, antes da conversão.
5. A simulação manual de teste que usa 99999 permanece com esse valor e
   `teste=true`; não é convertida em produção de 50 kg.

Isso é uma **estimativa operacional de 50 kg**, não prova de que o peso físico
era exatamente 50,000 kg. A regra pode ser desativada/ajustada nas constantes
`PESO_SATURADO_INDICADOR13` e `PESO_EQUIVALENTE_50KG13` em uma alteração futura.

## Outras duas correções apoiadas nos dados

**Par final consistente.** Em dois sacos de 25 kg, o indicador só forneceu duas
leituras finais próximas, insuficientes para mediana de três. A v13 conservava
uma estimativa mais antiga do enchimento. Agora o par final consistente vira
peso estimado e insuficientemente confirmado; não é promovido a estabilidade
ou mediana de três.

| Sequência real, bruta | v13 anterior | v13.1 |
|---|---:|---:|
| 252, 256, 273, 251, 251, 11, 6 | 256 estimado | **251 estimado** |
| 253, 273, 275, 251, 251, 14, 5 | 273 estimado | **251 estimado** |

O par precisa respeitar a tolerância configurada e não atravessa uma lacuna.
A estabilidade de duas amostras continua dependendo do bit do indicador; a
mediana de grupo continua exigindo três amostras. O exemplo original com
245 isolado durante a descarga continua preservando o patamar de 250, e um
patamar legítimo de 245 continua aceito.

**Retomada após intervalo no processamento.** Há um intervalo de 2,116 s no
log de 50 kg, entre os PKT de millis 3035189 e 3037305, durante enchimento com
SP3 desligado. Depois aparecem vários pacotes processados em rajada e uma
borda SP3 legítima, no ciclo 211. Na v13 anterior, a invalidação ociosa exigia
outra descarga antes de reconhecer essa borda e ignorava o saco.

A v13.1 preserva o estado armado quando uma descarga já foi confirmada antes,
reaprende o primeiro estado SP3 sem fabricar borda e aceita o próximo 0→1
efetivamente observado. Sinaliza a lacuna e a estimativa. Um boot novo ainda
exige duas leituras baixas para armar. Pausas seguidas de rajadas são compatíveis
com processamento de buffer após operações de rede; não comprovam falha física
da serial ou do indicador.

## Compatibilidade de persistência

O estado do motor permanece com 380 bytes, sem mudança de layout ou versão.
O byte de flags já existente no checkpoint guarda recuperação, saturação e
revisão; o layout do checkpoint também foi preservado. A fila continua no
formato v5 com CRC, e o ID persistente mantém o namespace `e13_...`.

O critério fica congelado por registro: `D` identifica registros v13 anteriores,
`d` os registros comuns novos e `L` a saturação. O envio preserva a identificação
de firmware de registros v13 já enfileirados; não muda payload/chave de um retry
antigo só por o programa agora ser v13.1. A confirmação remota continua usando
criação condicional e comparação do payload, sem sobrescrever históricos.

## Validação com os pacotes reais

Os testes usam o motor, o adaptador e a fila extraídos do `.ino` final, com
LittleFS/Firebase simulados. As fixtures CSV incluem apenas horário, millis,
peso, SP3, estabilidade e bytes STS1/STS2; não incluem dados de rede dos logs.

| Reprodução | Resultado v13.1 |
|---|---|
| 154 PKT de 25 kg | **5 eventos**, pesos 251, 251, 249, 251 e 251 |
| 753 PKT de 50 kg, limite bruto 200 | **13 eventos completos recuperáveis**, todos 500 estimados por saturação |
| Mesmo log de 50 kg com o rótulo 25 kg do cabeçalho, limite 100 | mesmos 13 eventos completos |

Os 13 eventos não são uma contagem total do arquivo inteiro. O monitor PKT
começa no meio de um saco e foi desligado em outro trecho, com intervalo de
28,829 s sem PKT. Esses intervalos não fornecem informação suficiente para
reconstituir todos os ensaques e não são tratados como perda serial comprovada.

Também foram mantidos os testes anteriores de SP3 oscilando, sacos separados
por 6 segundos, CRC, IDs, filas legadas, falta de internet, corte entre append e
checkpoint, escrita parcial, backlog cheio e proteção de registros históricos.
Os novos testes incluem saturação no início do ciclo, saturação antes de formar
trio, reinício após saturação e preservação da simulação 99999.

Execute tudo com `python3 tests/v13_1/run_tests.py`. Não há envio de dados de
teste para o Firebase real. Logs e manifesto da revisão acompanham a entrega
em `docs/evidencias/v13_1/`.

Resultado final: **51 grupos aprovados**, distribuídos entre 15 do motor,
17 da fila (1.562 assertivas), 16 do adaptador (2.732 assertivas) e 3 de
reprodução dos logs (4.391 assertivas), com ASan/UBSan sem erros.

## Compilação e limites de campo

A v13.1 compilou para LOLIN(WeMos) D1 R1, ESP8266 core 3.1.2, com as mesmas
quatro bibliotecas, pinagens e placeholders de credenciais da v12. RAM estática:
49.648/80.192 bytes (61%); IRAM incluindo cache: 61.967/65.536 (94%); código em
flash: 633.968/1.048.576 (60%). Os avisos são os já conhecidos do `elf2bin.py`
do core usando Python 3.12, sem erro de compilação.

Ainda é necessário validar na máquina a indicação de limite/saturação, a casa
decimal, a seleção do produto, LittleFS real sob corte de energia, uso dinâmico
de memória, periféricos e envio autenticado. O log menciona um último reset
por exceção 29 no login Firebase; isso não demonstra que a v13.1 reinicie nem
prova que o problema de memória esteja resolvido. O teste físico continua
necessário. Não apagar a flash/LittleFS nem mudar FS:2MB para fazer um teste.

As demais limitações e o desenho da fila estão no relatório original
`docs/RELATORIO_V13.md`. Nenhum registro histórico foi alterado.

### Limitação adicional identificada na revisão comparativa

O parser e o motor atuais da v13.1 rejeitam pesos negativos como `-0002`.
Se a descarga terminar com duas leituras negativas e não houver duas leituras
não negativas abaixo do limite, o ciclo pode continuar aberto. O caso foi
reproduzido em teste isolado; a versão publicada ainda precisa receber esse
tratamento antes de ser usada em uma instalação que apresente pesos negativos
depois da retirada do saco. Os 51 grupos aprovados não cobriam esse cenário.

Esta observação atualiza somente a documentação: o `.ino` publicado permanece
idêntico ao arquivo compilado e testado, sem correção silenciosa de firmware.

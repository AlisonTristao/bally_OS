# Playlist de corrida para a Junkebox

Os dez arquivos desta pasta usam diretamente o formato aceito por
`lib/Junkebox`: uma nota e sua duracao em milissegundos por linha. Todos os
arranjos sao monofonicos, duram aproximadamente um minuto e continuam menores
que o limite de 4096 bytes da biblioteca gracas a diretiva `LOOP,N`.

## Faixas

1. `01_turbo_frog.txt` - tema eletronico original, rapido e agudo
2. `02_william_tell_finale.txt` - Rossini, final da abertura William Tell
3. `03_flight_of_the_bumblebee.txt` - Rimsky-Korsakov
4. `04_can_can.txt` - Offenbach, Galop Infernal
5. `05_ride_of_the_valkyries.txt` - Wagner
6. `06_beethoven_5.txt` - Beethoven, Sinfonia No. 5
7. `07_turkish_march.txt` - Mozart, Rondo Alla Turca
8. `08_korobeiniki.txt` - cancao folclorica russa conhecida pelo Tetris
9. `09_mountain_king.txt` - Grieg, In the Hall of the Mountain King
10. `10_toccata_in_d_minor.txt` - Bach, Toccata em Re menor

`Turbo Frog` e uma composicao nova para dar a sensacao acelerada e brincalhona
de temas eletronicos de corrida. Ela nao reproduz a melodia de `Axel F`/Crazy
Frog. As outras nove sao arranjos simplificados de composicoes em dominio
publico.

## Como usar

Copie os dez `.txt` para a raiz do cartao SD. Depois ejete o volume com
seguranca no PC e pressione o botao 2 do robo para devolver o cartao ao
firmware. `storage -status` deve responder `robot owns SD`; enquanto responder
`PC owns SD`, somente os sons builtin funcionam.

Com o cartao sob controle do robo, confirme e toque uma faixa pelo shell:

```text
storage -exists 01_turbo_frog.txt
junkebox -list
junkebox -play 01_turbo_frog.txt
junkebox -play 02_william_tell_finale.txt
junkebox -stop
```

Tambem e possivel agendar uma faixa:

```text
job -once 1000, junkebox -play 01_turbo_frog.txt
```

As linhas iniciadas por `#` sao comentarios. `REST` cria silencio e `LOOP,10`
repete o arquivo completo dez vezes (limite de 100). Para editar ou compor
outra musica, use apenas `A` a `G`, um `#` ou `b` opcional e a oitava, como
`C4`, `F#5` ou `Bb4`.

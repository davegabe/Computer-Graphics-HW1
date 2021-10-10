
# Computer Graphics: HW1
Implementazione di filtri per immagini utilizzando la libreria [**Yocto/GL**](https://github.com/xelatihy/yocto-gl).

# Funzionalità richieste

- **Tone mapping**
  - Expoure compensation
  - Filmic correction (fit del tonemapping cinematografico ACES)
  - sRGB color space
- **Color tint**
- **Saturation**
- **Contrast**
- **Vignette**
- **Film grain**
- **Mosaic Effect**
- **Grid Effect**

# Funzionalità extra

## Sketch
Il filtro simula un effetto disegnato alla foto a cui viene applicato.
![toa_heftiba_people.sketch](extra/toa_heftiba_people.sketch.jpg)
![greg_zaal_artist_workshop.sketch](extra/greg_zaal_artist_workshop.sketch.jpg)
Source: https://www.shadertoy.com/view/7scSRX

## VHS
Il filtro simula un effetto videocassetta alla foto a cui viene applicato.
![toa_heftiba_people.sketch](extra/toa_heftiba_people.vhs.jpg)
![greg_zaal_artist_workshop.sketch](extra/greg_zaal_artist_workshop.vhs.jpg)
Source: https://www.shadertoy.com/view/Mt3yz4

## Attribute clash
Il filtro simula gli artefatti grafici noti come "[attribute clash](https://en.wikipedia.org/wiki/Attribute_clash)" alla foto a cui viene applicato.
![toa_heftiba_people.sketch](extra/toa_heftiba_people.clash.jpg)
![greg_zaal_artist_workshop.sketch](extra/greg_zaal_artist_workshop.clash.jpg)
Source: https://www.shadertoy.com/view/XsfcD8

## Negative
Il filtro simula un di pellicola negativa alla foto a cui viene applicato.
![toa_heftiba_people.sketch](extra/toa_heftiba_people.negative.jpg)
![greg_zaal_artist_workshop.sketch](extra/greg_zaal_artist_workshop.negative.jpg)

## Mirror
Il filtro simula un effetto specchio in un certo punto dell'immagine dato in input. Se viene dato il valore 50% (come nella foto) specchia esattamente la foto a metà. Se vengono dati valori inferiori al 50% allora l'immagien verrà replicata più volte con lo stesso pattern, creando effetti più creativi.
![toa_heftiba_people.sketch](extra/toa_heftiba_people.mirror.jpg)
![greg_zaal_artist_workshop.sketch](extra/greg_zaal_artist_workshop.mirror.jpg)
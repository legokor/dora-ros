Hogy lehessen devcontainerezni, le kell buildelni
A "docker-multistage" helyett persze kicserélni a megfelelő branch-re:
do build -t dora-local-devcon --build-arg GIT_BRANCH=docker-multistage .

Most a .devcontainer úgy van bekötve, hogy online helyett lokális konténerbe tölti be magát
Belépés után a github nem fog működni auth miatt, nálam már megy, de nagyon nehezen.
Az ssh kulcs csak be van mountolva a konténerbe, ez nálam id_rsa.pub, mindenkinél más-más, windows meg windows.
Ha nem id_rsa akkor át kell írni a .devcontainer-ben.

És hozzá kell adni giten:
Devcontaineren belül ki kell íratni, és hozzáadni neten a kulcsot (ssh-ed25519-val kezdődik)
	cat ~/.ssh/id_rsa.pub
Github -> Settings -> SSH and GPG keys -> New SSH key
El kell nevezni, és hozzáadni

Csomagokat a Dockerfile-hoz hozzáadni a megfelelő helyen, a listába kell beleírni.
Most mindent mindenhova telepít, alapértelmezetten a rosdep paranccsal feltérképezné a csomagokat, és automatikusan kiszedné belőle a függőségeket, ehhez adnánk hozzá a sajátot. Most örülök, hogy így menni látszik.

És nincsen prod stage se, mert nincs értelme, hogy a packages.xml-ben nincs rendesen szétszedve a build és run dependency.
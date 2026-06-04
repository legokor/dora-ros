Ebben a file-ban vannak mindenféle parancsok aktív fejlesztéshez, meg hogy mi változott

do build -t test:1 -f testing.Dockerfile --target test1 .

do run --rm -it test:1 bash

rviz és dev containerek a builder-ből származtatva, mert valszeg építünk ezekből

package lista megtartása
	TODO: kiszedni prod-ból

builder stage behozása, ebből épül a dev és rviz stage, prod innen másol

Kéne: Scriptek összerakva, felírva, mi mit csinál (auto source, stb)

builder stage tényleg lebuildeli, nem csak amikor target

sajnos emiatt a dev meg ilyesmikbe is lesz build

nano letöltése, mint terminal szövegszerkesztő

új no-build-run.sh, mert az eddigi run.sh buildel is
marad a run.sh, mert nagyon jó lenne, ha nem jönnénk rá, hogy nem épít a script ami eddig épített
azért no-build-run ahelyett, hogy run-no-build, hogy lehessen tabolni a régit is

Változtatások miatt most a dev stage alapparancsa csak lebuildeli

Még kérdések: pontosan mi elég a robot futtatásához?
Pl a package-ek közül nem kell a mindenféle szövegszerkesztő, ros-dev-tools, stb stb
De nem akarok későbbi stage-ekben mozgatni.

Git dolgok:
Jelenleg az action minden push-nál csak a main branchet építi le.

Most van rá egy build arg, amit minden stage-nél aminél használjuk, be kell hozni ARG-al
--build-arg GIT_BRANCH=branchname-el lehet buildelni a megfelelőt

Ezt elkezdem felülvizsgálni

Valami valami ha más branch unshallow van, megnézni
https://stackoverflow.com/questions/39957760/how-to-fetch-all-remote-branches
(Note: If you combined --depth 1 with a single branch originally, you may need to run git config remote.origin.fetch "+refs/heads/*:refs/remotes/origin/*" right before running --unshallow to ensure it grabs the history for all branches).

Beleraktam Misi csomagjait is a dev stage-be, dev stage végén próbál a githubbal rendesen csatlakozni

Még meg kell oldani a github authot, de lehet, ha megy a devcontainerrel, akkor csak hagyom.

Lehet érdemesebb lenne a build-et a devtől különválasztani, mert branchváltásnál várni kell két percet a dev csomagok lehúzására.
Szerintem ezt nem tudja kijavítani semmi se (változik egy egész stage a dev előtt)
Most csak beleraktam a dev stage-be az építést, és kezdem az egész vonalat a base-től

new.Dockerfile: még nem kezdtem el, majd itt lesz a tutorial alapján történő caches átírás
csak előtte szeretném, ha pl a branchválasztás rendesen menne




TMP:
do build -t dora-test:base --target base . && \
do run --rm -it dora-test:base bash

do build -t dora-test:prod --target prod . && \
do run --rm -it dora-test:prod bash


do build -t dora-test:prod --build-arg GIT_BRANCH=docker-multistage --target prod . && \
do run --rm -it dora-test:prod bash

do build -t dora-test:prod --build-arg GIT_BRANCH=docker-multistage --target dev . && \
do run --rm -it dora-test:prod bash

do build -t newbuild -f new.Dockerfile . && \
do run --rm -it newbuild bash

do build -t dora-test:prod --build-arg GIT_BRANCH=docker-multistage --target dev -f new.Dockerfile . && \
do run --rm -it dora-test:prod bash



Decon github auth notes:
Végül csak bemountolva a devcontainer leírásban az ssh kulcs.
Ezt viszont hozzá kell adni giten:
Devcontaineren belül ki kell másolni a kulcsot (ssh-ed25519-val kezdődik)
	cat ~/.ssh/id_rsa.pub
Github -> Settings -> SSH and GPG keys -> New SSH key
El kell nevezni, és hozzáadni
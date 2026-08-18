#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Serveur statique du cockpit — remplaçant de `python3 -m http.server 8000`.

Différence unique : les documents qui changent à chaque mise à jour partent
avec `Cache-Control: no-cache`, pour qu'ils soient TOUJOURS revalidés. Fini
les pages périmées après chaque mise à jour du site (l'utilisateur voyait des
textes vieux de plusieurs versions) ; les assets versionnés (?v=) restent
cachables.

Couvre .html/.json ET le rapport PDF (2026-07-29). Le PDF avait été oublié
lors de la correction initiale, et le symptôme était le même à un étage
au-dessus : le tunnel Cloudflare, ne voyant aucun `Cache-Control` sur un
.pdf, appliquait son défaut de 4 h (`cf-cache-status: HIT`, `age: 5256`) et
servait publiquement un rapport de 300 pages alors que le disque en portait
309. Un rechargement forcé du navigateur n'y pouvait rien : le cache était
au bord du réseau, pas chez le client.
"""
import http.server
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))


class NoHtmlCacheHandler(http.server.SimpleHTTPRequestHandler):
    # ── Ce qui n'est PAS servi (audit du 2026-08-18) ────────────────────────
    # `SimpleHTTPRequestHandler` sert TOUT le repertoire, listings compris, et
    # ce repertoire est publie sur internet par le tunnel Cloudflare. L'audit a
    # verifie en direct que `https://cockpit.leo-rover-gardon.dev/exports/`
    # repondait 200 avec l'index navigable de 104 fichiers (17 Mo de donnees
    # d'essai : .mat, CSV de trajectoires, figures), idem `models/`,
    # `.bak-cdn-20260729/` et les huit pages `.bak-cachefix-*.html` du 29/07 —
    # des copies perimees du cockpit, servies publiquement a cote des vraies.
    # Deux regles suffisent a fermer ca sans rien casser :
    #   1. tout segment de chemin commencant par un point -> 404 (les
    #      sauvegardes du projet sont nommees `.bak-*`, cf. section 10 du
    #      .gitignore, et ca couvre aussi un `.git` qui atterrirait ici) ;
    #   2. plus aucun listing de repertoire -> 404. `send_head()` continue de
    #      servir `index.html` quand il existe, donc `/` reste intact ; seuls
    #      les repertoires SANS index deviennent muets, ce qui est exactement
    #      le cas de exports/, models/, vendor/, fonts/, img/ et reports/.
    #      Les fichiers qu'ils contiennent restent accessibles par leur URL
    #      directe (le lien du rapport PDF, les polices, les scripts) : c'est
    #      l'enumeration qui disparait, pas le service.
    # Les motifs ci-dessous sont ceux de la section 10 du .gitignore, a
    # dessein : ce que git refuse d'archiver, le serveur refuse de publier.
    # Un point de depart « tout segment commencant par un point » ne suffit
    # PAS — la convention du projet produit aussi des noms comme
    # `ops.html.bak-avant-v4-161908` ou `app.js.bak-casse-203841`, qui sont
    # des cockpits complets d'anciennes versions et ne commencent par aucun
    # point. L'audit en a trouve 13 de cette forme, servis publiquement.
    MOTIFS_REFUSES = (".bak", ".orig", ".rej", ".swp")

    def _refuse(self, chemin):
        for seg in chemin.split("/"):
            if not seg:
                continue
            if seg.startswith("."):
                return True
            for motif in self.MOTIFS_REFUSES:
                # couvre `x.bak`, `x.bak-horodatage` et `x.bak_horodatage`
                if motif in seg and seg.split(motif, 1)[1][:1] in ("", "-", "_"):
                    return True
        return False

    def send_head(self):
        chemin = self.path.split("?")[0].split("#")[0]
        if self._refuse(chemin):
            self.send_error(404, "Not Found")
            return None
        return http.server.SimpleHTTPRequestHandler.send_head(self)

    def list_directory(self, path):
        self.send_error(404, "Not Found")
        return None

    def end_headers(self):
        p = self.path.split("?")[0]
        # Le rapport est régénéré à chaque avancée : il doit être revalidé au
        # même titre que le HTML, sinon Cloudflare le fige (voir en-tête).
        # .js/.css AJOUTES (2026-07-29). Le projet versionne ces fichiers par
        # `?v=rNN` et comptait sur cette estampille seule. Deux defauts se sont
        # manifestes le meme jour : (1) modifier i18n.js/app.js SANS incrementer
        # l'estampille laissait le navigateur servir sa copie en cache — les
        # cles i18n s'affichaient en brut et un app.js perime faisait croire
        # que le bouton Export etait casse ; (2) les estampilles avaient DERIVE
        # entre pages (app.js en r28 sur deux pages, r30 sur une autre), donc
        # chaque page cachait une version differente. `no-cache` ne desactive
        # pas le cache : il force une REVALIDATION, donc un 304 bon marche.
        # L'estampille reste utile pour l'invalidation immediate ; ceci est le
        # filet qui rend l'oubli inoffensif.
        volatil = (p.endswith((".html", ".json", ".pdf", ".js", ".css"))
                   or p.startswith("/reports/"))
        if volatil or p.endswith("/") or "." not in p.rsplit("/", 1)[-1]:
            self.send_header("Cache-Control", "no-cache, must-revalidate")
        http.server.SimpleHTTPRequestHandler.end_headers(self)

    def log_message(self, *args):
        pass  # silencieux (le log http n'a jamais servi au debug)


if __name__ == "__main__":
    http.server.ThreadingHTTPServer(("0.0.0.0", 8000), NoHtmlCacheHandler).serve_forever()

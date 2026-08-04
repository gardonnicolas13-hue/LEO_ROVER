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

# Aggiungi una scorciatoia per "uscire" (per chiudere la sessione tmux)
bind -n C-q kill-session  # Ctrl+q per uccidere la sessione corrente
bind -n C-F12 confirm-before -p "Chiudere tutte le sessioni tmux? (y/n)" "run-shell 'tmux kill-server'"
# Passaggio rapido tra sessioni
bind -n F1 switch-client -t camera
bind -n F2 switch-client -t object_detection
bind -n F3 switch-client -t tool_detection

# Mostra messaggio sulla barra per conferma
# Passaggio rapido tra sessioni con messaggio di conferma
bind -n F1 run-shell "tmux switch-client -t camera; tmux display-message 'Switched to session: camera'"
bind -n F2 run-shell "tmux switch-client -t object_detection; tmux display-message 'Switched to session: object_detection'"
bind -n F3 run-shell "tmux switch-client -t tool_detection; tmux display-message 'Switched to session: tool_detection'"

# Mostra i comandi nella status bar
# Impostare i colori della barra di stato
set -g status-style bg='#2e3440',fg='#d8dee9'

# Barra di stato sinistra con scorciatoie e messaggi
set -g status-left '#[fg=yellow] F1:camera #[fg=green] F2:obj_det #[fg=magenta] F3:tool_det #[fg=red] Ctrl+Q:quit #[fg=red] Ctrl+F12:exit_all            '

# Impostare la lunghezza della barra di stato
set -g status-left-length 90

# Mouse
set -g mouse on

nano ~/.tmux.conf
tmux source-file ~/.tmux.conf